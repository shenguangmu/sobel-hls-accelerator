#!/usr/bin/env tclsh
# =====================================================================
#  run_cosim.tcl —— C/RTL 协同仿真（没有板子也能验证 RTL 行为）
#
#  它做什么：
#    csim  → csynth → cosim
#    先用 C 跑一遍 testbench 生成"黄金向量"，然后在真实的 RTL 仿真器
#    （XSIM）里重放同一份 testbench，逐拍驱动生成的 Verilog 模块，
#    再和 C 侧结果比对。结论以
#        INFO: [COSIM 212-1000] *** C/RTL co-simulation finished: PASS ***
#    为准。
#
#  为什么值得跑：
#    - csim 只验证 C 语义；cosim 验证**综合后的 RTL 时序行为**，
#      能抓到流水线握手、反压、TLAST 位置这类只有硬件才暴露的问题。
#    - 也是上板前最接近真实的一块拼图。
#
#  用法（在仓库根目录）：
#    vitis-run --mode hls --tcl src_hls/run_cosim.tcl        # 2024.2+
#    vitis_hls -f src_hls/run_cosim.tcl                      # 2023.2 及更早
#
#  可选参数用环境变量（--tcl 后面不能再跟参数）：
#    SOBEL_COSIM_ONLY=csim|csynth|cosim   只跑某一步
#    SOBEL_COSIM_PART=<part>              默认 xc7z020clg400-1
#    SOBEL_COSIM_CLOCK=<ns>               默认 10.0
#    SOBEL_COSIM_RTL=verilog|vhdl         默认 verilog
# =====================================================================

set argv [list]
set argc 0

# ---- 定位仓库根（脚本在 src_hls/ 下，上一级就是根）----
set SCRIPT_DIR [file normalize [file dirname [info script]]]
if {[file exists "$SCRIPT_DIR/sobel_hls.cpp"]} {
    set SRC_DIR  $SCRIPT_DIR
    set PROJ_ROOT [file normalize "$SCRIPT_DIR/.."]
} else {
    # 从仓库根调用时，脚本路径可能已被解析成相对形式
    set PROJ_ROOT [file normalize [pwd]]
    set SRC_DIR   "$PROJ_ROOT/src_hls"
}

set VEC_DIR  [file normalize "$PROJ_ROOT/sim/vectors"]
set PART     "xc7z020clg400-1"
set CLOCK_NS "10.0"
set RTL_LANG "verilog"
set ONLY     "all"

foreach {envvar var} {
    SOBEL_COSIM_PART  PART
    SOBEL_COSIM_CLOCK CLOCK_NS
    SOBEL_COSIM_RTL   RTL_LANG
    SOBEL_COSIM_ONLY  ONLY
} {
    if {[info exists ::env($envvar)]} { set $var $::env($envvar) }
}

puts "====================================================================="
puts "  Sobel C/RTL 协同仿真"
puts "  仓库根   : $PROJ_ROOT"
puts "  向量目录 : $VEC_DIR"
puts "  器件     : $PART"
puts "  时钟约束 : ${CLOCK_NS} ns"
puts "  RTL 语言 : $RTL_LANG"
puts "  执行步骤 : $ONLY"
puts "====================================================================="

# 向量目录必须存在，否则外部向量那 6 组会被跳过（仍会 PASS，但覆盖变弱）
if {[llength [glob -nocomplain "$VEC_DIR/*_in.txt"]] == 0} {
    puts "\n!!! 向量目录是空的：$VEC_DIR"
    puts "    请先在仓库根执行: python sim/gen_vectors.py"
    puts "    （不跑也能过，但只有自一致性的 8 组，覆盖不到 Python golden）\n"
}

# 生成向量路径头文件，让 testbench 在深层构建目录里也能找到向量
set fh [open "$SRC_DIR/tb_paths.h" w]
puts $fh "/* 由 run_cosim.tcl 自动生成，勿手改 */"
puts $fh "#ifndef TB_PATHS_H"
puts $fh "#define TB_PATHS_H"
puts $fh "#define TB_VEC_DIR_DEFAULT \"[string map {\\ /} $VEC_DIR]\""
puts $fh "#endif"
close $fh

# ---- 建工程 ----
open_project -reset sobel_cosim
set_top sobel_accel
add_files    -cflags "-I$SRC_DIR" "$SRC_DIR/sobel_hls.cpp"
add_files -tb -cflags "-I$SRC_DIR" "$SRC_DIR/tb_sobel.cpp"

open_solution -reset "sol1" -flow_target vivado
set_part $PART
create_clock -period $CLOCK_NS -name default

# ---- 1. C 仿真 ----
if {$ONLY eq "all" || $ONLY eq "csim"} {
    puts "\n>>> C 仿真"
    if {[catch { csim_design } err]} {
        puts "\n!!! C 仿真失败:\n$err"; close_project; exit 1
    }
    puts ">>> C 仿真通过"
}

# ---- 2. C 综合 ----
if {$ONLY eq "all" || $ONLY eq "csynth"} {
    puts "\n>>> C 综合"
    if {[catch { csynth_design } err]} {
        puts "\n!!! C 综合失败:\n$err"; close_project; exit 1
    }
    puts ">>> C 综合完成"
    set rpt "sobel_cosim/sol1/syn/report/sobel_accel_csynth.rpt"
    if {[file exists $rpt]} {
        set f [open $rpt r]
        while {[gets $f line] >= 0} {
            if {[regexp {(Estimated Fmax|Final II|Target II)} $line]} {
                puts "    [string trim $line]"
            }
        }
        close $f
    }
}

# ---- 3. C/RTL 协同仿真 ----
if {$ONLY eq "all" || $ONLY eq "cosim"} {
    puts "\n>>> C/RTL 协同仿真（这一步会调用 XSIM 跑真实 RTL，较慢）"
    if {[catch { cosim_design -rtl $RTL_LANG -trace_level none } err]} {
        puts "\n!!! 协同仿真失败:\n$err"
        close_project
        exit 1
    }
    puts "\n>>> 协同仿真通过"
}

close_project
puts "\n====================================================================="
puts "  完成。结论以日志里的"
puts "    *** C/RTL co-simulation finished: PASS ***"
puts "  为准 —— 只看到 \"finished successfully\" 不代表比对通过。"
puts "====================================================================="
