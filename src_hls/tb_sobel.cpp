/**
 * @file    tb_sobel.cpp
 * @brief   Sobel 加速器的 C 仿真 testbench
 *
 * 顶层是 hls::stream 接口：先把 width*height 个输入像素灌进 in 流，
 * 再从 out 流里收同样多个输出像素。单线程，原因见 run_hw() 的注释
 * （要点：C/RTL 协同仿真禁止"空流读取"，并发模型会被它判为致命错误）。
 *
 * 两段式验证（与旧版一致）:
 *   [1] 自一致性 —— sobel_accel（流式）vs sobel_ref_sw（朴素逐像素）
 *   [2] 外部向量 —— sobel_accel vs Python golden（sim/vectors/）
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

#include "sobel_hls.h"

/* tb_paths.h 由 run_hls.tcl / run_cosim.tcl 生成，
 * 里面是向量目录的绝对路径（TB_VEC_DIR_DEFAULT）。
 * 用 __has_include 保护 —— 手动编译、没生成该文件时也不会报错。 */
#if defined(__has_include)
#  if __has_include("tb_paths.h")
#    include "tb_paths.h"
#  endif
#endif

/* sobel_hls.cpp 里定义的仿真专用参考实现 */
#ifndef __SYNTHESIS__
void sobel_ref_sw(const ap_uint<8> *src, ap_uint<8> *dst,
                  int width, int height, int thresh, int gain);
#endif

/* 工作缓冲（主机内存，不受 HLS 约束） */
static std::vector<ap_uint<8> > g_in;
static std::vector<ap_uint<8> > g_hw;
static std::vector<ap_uint<8> > g_sw;

/* ------------------------------------------------------------------ *
 *  一次完整的流式调用
 * ------------------------------------------------------------------ */

/**
 * @brief 把 g_in 的 width*height 个像素喂给 sobel_accel，收齐输出到 g_hw
 *
 * 先灌完输入、再收输出 —— 单线程，且**不能改成并发**。
 *
 * 为什么不能用生产者/消费者双线程:
 *   C/RTL 协同仿真（cosim_design）在生成测试向量那一阶段，
 *   会把"空流被读取"判为致命错误：
 *     ERROR [HLS SIM]: an hls::stream is read while empty,
 *                      which may result in RTL simulation hanging.
 *   并发模型下这个窗口几乎必然出现，cosim 直接中止。
 *   （加 -DALLOW_EMPTY_HLS_STREAM_READS 会把它降级成警告，
 *     但那会让读变成非阻塞，双线程模型反而算错，本文件试过。）
 *
 * 为什么单线程不会死锁:
 *   hls::stream 在 C 仿真里是**无界队列**（实测 640x480 时
 *   "maximum depth reached ... is 307200" = 一帧全部像素）。
 *   所以先把输入灌完不会阻塞；随后 sobel_accel 消费输入、
 *   产出输出，输出同样进无界队列，也就不存在背压死锁。
 *
 * 代价：内存占用 = 2 x 帧大小，对大图（如 1080p）约 4 MB，可接受。
 */
static void run_hw(int w, int h, int thresh, int gain)
{
    const size_t n = (size_t)w * h;

    hls::stream<axis_word_t> s_in;
    hls::stream<axis_word_t> s_out;

    g_hw.assign(n, 0);

    for (size_t i = 0; i < n; i++) {
        axis_word_t wd;
        wd.data = g_in[i];
        wd.keep = 1;
        wd.strb = 1;
        wd.last = 0;         /* 输入侧 TLAST 对本设计无意义 */
        s_in.write(wd);
    }

    sobel_accel(s_in, s_out, w, h, thresh, gain);

    /* 设计会输出恰好 width*height 个像素（输出图与输入同尺寸）。
     * 顺带校验 TLAST：只应出现在最后一个像素上。 */
    int tlast_count = 0;
    for (size_t i = 0; i < n; i++) {
        axis_word_t wd = s_out.read();
        g_hw[i] = wd.data;
        if (wd.last) {
            tlast_count++;
            if (i != n - 1) {
                std::cout << "    [警告] TLAST 出现在第 " << i
                          << " 个像素（应为 " << (n - 1) << "）\n";
            }
        }
    }
    if (tlast_count != 1) {
        std::cout << "    [警告] TLAST 数量 = " << tlast_count
                  << "（应为 1）\n";
    }
}

/* ------------------------------------------------------------------ *
 *  测试图生成
 * ------------------------------------------------------------------ */

static void gen_gradient(int w, int h)
{
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            g_in[y * w + x] = (ap_uint<8>)((x * 8) & 0xFF);
}

static void gen_checker(int w, int h, int block)
{
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            int c = (((y / block) + (x / block)) & 1) ? 255 : 0;
            g_in[y * w + x] = (ap_uint<8>)c;
        }
}

static void gen_shapes(int w, int h)
{
    for (int i = 0; i < w * h; i++) g_in[i] = 0;
    for (int y = 8; y < 24 && y < h; y++)
        for (int x = 8; x < 24 && x < w; x++)
            g_in[y * w + x] = 200;
    for (int y = 36; y < 56 && y < h; y++)
        for (int x = 36; x < 60 && x < w; x++)
            g_in[y * w + x] = 128;
    for (int i = 0; i < w && i < h; i++)
        g_in[i * w + (w - 1 - i)] = 255;
}

static void gen_ramp(int w, int h)
{
    int band = w / 8;
    if (band < 1) band = 1;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            g_in[y * w + x] = (ap_uint<8>)(((x / band) * 32) & 0xFF);
}

static void gen_noise(int w, int h, unsigned seed)
{
    unsigned s = seed;
    for (int i = 0; i < w * h; i++) {
        s = s * 1103515245u + 12345u;
        g_in[i] = (ap_uint<8>)((s >> 16) & 0xFF);
    }
}

/* ------------------------------------------------------------------ *
 *  向量文件
 * ------------------------------------------------------------------ */

static bool load_vec(const std::string &path,
                     int &w, int &h, std::vector<int> &data)
{
    std::ifstream f(path);
    if (!f.is_open()) return false;
    if (!(f >> w >> h) || w <= 0 || h <= 0) return false;
    if ((long)w * h > (long)SOBEL_MAX_WIDTH * SOBEL_MAX_HEIGHT) return false;
    data.resize((size_t)w * h);
    for (size_t i = 0; i < data.size(); i++)
        if (!(f >> data[i])) return false;
    return true;
}

/* ------------------------------------------------------------------ *
 *  比较与报告
 * ------------------------------------------------------------------ */

static void compare_and_report(const char *tag,
                               const std::vector<ap_uint<8> > &got,
                               const std::vector<ap_uint<8> > &exp,
                               int w, int &fails)
{
    ptrdiff_t first_bad = -1;
    size_t n_bad = 0;
    const size_t n = exp.size();

    for (size_t i = 0; i < n; i++) {
        if ((int)got[i] != (int)exp[i]) {
            if (first_bad < 0) first_bad = (ptrdiff_t)i;
            n_bad++;
        }
    }

    if (n_bad == 0) {
        std::cout << "    " << tag << " : PASS\n";
        return;
    }

    fails++;
    std::cout << "    " << tag << " : FAIL  " << n_bad << "/" << n
              << " 个像素不符\n";
    std::cout << "        首个: x=" << (first_bad % w)
              << " y=" << (first_bad / w)
              << "  exp=" << (int)exp[first_bad]
              << " got=" << (int)got[first_bad] << "\n";

    int shown = 0;
    for (size_t i = 0; i < n && shown < 6; i++) {
        if ((int)got[i] != (int)exp[i]) {
            std::cout << "          [" << i << "] x=" << (i % w)
                      << " y=" << (i / w)
                      << " exp=" << (int)exp[i]
                      << " got=" << (int)got[i] << "\n";
            shown++;
        }
    }
}

/* ------------------------------------------------------------------ *
 *  单组自一致性测试
 * ------------------------------------------------------------------ */

static int test_self(const char *name, int w, int h,
                     int thresh, int gain, void (*gen)(int, int))
{
    const size_t n = (size_t)w * h;
    g_in.assign(n, 0);
    gen(w, h);

    std::vector<ap_uint<8> > sw(n, 0);
    sobel_ref_sw(g_in.data(), sw.data(), w, h, thresh, gain);
    run_hw(w, h, thresh, gain);

    std::cout << "  [" << name << "] " << w << "x" << h << "\n";
    int f = 0;
    compare_and_report("hw vs sw", g_hw, sw, w, f);
    return f;
}

/* ------------------------------------------------------------------ *
 *  单组外部向量测试（返回 -1 表示跳过）
 * ------------------------------------------------------------------ */

static int test_vec(const std::string &dir, const char *name,
                    int thresh, int gain)
{
    int w = 0, h = 0;
    std::vector<int> in, gold;

    if (!load_vec(dir + "/" + name + "_in.txt", w, h, in))     return -1;
    if (!load_vec(dir + "/" + name + "_gold.txt", w, h, gold)) return -1;

    const size_t n = (size_t)w * h;
    g_in.assign(n, 0);
    for (size_t i = 0; i < n; i++) g_in[i] = (ap_uint<8>)in[i];

    run_hw(w, h, thresh, gain);

    std::vector<ap_uint<8> > exp(n);
    for (size_t i = 0; i < n; i++) exp[i] = (ap_uint<8>)gold[i];

    std::cout << "  [" << name << "] " << w << "x" << h << " (向量)\n";
    int f = 0;
    compare_and_report("hw vs python", g_hw, exp, w, f);
    return f;
}

/* ------------------------------------------------------------------ *
 *  main
 * ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
    /* 向量目录的确定顺序：
     *   1. TB_VEC_DIR 环境变量（运行时覆盖，最灵活）
     *   2. TB_VEC_DIR_DEFAULT —— 由 run_hls.tcl 生成的 tb_paths.h 提供
     *   3. 相对路径 "sim/vectors"（在项目根手动编译时）
     *
     * 若向量目录不存在，依次尝试若干常见相对位置。
     * 这样纯 GUI 流程（不跑 run_hls.tcl、没有 tb_paths.h）也能工作。 */
    const char *env = std::getenv("TB_VEC_DIR");
    std::string vec_dir;
    bool found = false;
    if (env) {
        vec_dir = env;
        found = true;
    } else {
#ifdef TB_VEC_DIR_DEFAULT
        vec_dir = TB_VEC_DIR_DEFAULT;
        found = true;
#else
        /* 没有编译期注入时，从当前工作目录逐级往上找 sim/vectors。
         *
         * 为什么要这样：csim 是在某个深层构建目录里跑的，具体几层
         * 取决于工具版本和工程布局，写死相对路径很脆。
         * 逐级探测对任何布局都成立。
         *
         * 找不到也不致命 —— 第 [1] 段自一致性测试不依赖向量文件，
         * 第 [2] 段会自己跳过。 */
        std::string prefix;
        for (int i = 0; i < 8; i++) {
            std::string probe = prefix + "sim/vectors/gradient_in.txt";
            std::ifstream f(probe.c_str());
            if (f.is_open()) {
                vec_dir = prefix + "sim/vectors";
                found = true;
                break;
            }
            prefix += "../";
        }
        if (!found) vec_dir = "sim/vectors";
#endif
    }
    (void)found;

    int thresh = (argc > 1) ? std::atoi(argv[1]) : SOBEL_DEFAULT_THRESH;
    int gain   = (argc > 2) ? std::atoi(argv[2]) : SOBEL_DEFAULT_GAIN;

    std::cout << "========================================================\n";
    std::cout << "  Sobel HLS Testbench\n";
    std::cout << "  接口类型 : hls::stream (含 TLAST)\n";
    std::cout << "  thresh   = " << thresh << "   gain = " << gain << " (Q8)\n";
    std::cout << "  向量目录 = " << vec_dir << "\n";
    std::cout << "========================================================\n";

    int fails = 0;

    /* ---- 第一段：自一致性 ---- */
    std::cout << "\n[1] 自一致性：流式实现 vs 朴素实现\n";
    fails += test_self("gradient", 64, 48, thresh, gain, gen_gradient);
    fails += test_self("checker",  64, 48, thresh, gain,
                       [](int w, int h){ gen_checker(w, h, 4); });
    fails += test_self("shapes",   64, 64, thresh, gain, gen_shapes);
    fails += test_self("ramp",     64, 48, thresh, gain, gen_ramp);
    fails += test_self("noise",    61, 47, thresh, gain,
                       [](int w, int h){ gen_noise(w, h, 12345u); });
    fails += test_self("single",   1,  1,  thresh, gain, [](int, int){});
    fails += test_self("thin",     1, 32,  thresh, gain,
                       [](int w, int h){
                           for (int i=0;i<w*h;i++) g_in[i]=(ap_uint<8>)(i*7);
                       });
    fails += test_self("flat",     33, 33, thresh, gain,
                       [](int w, int h){
                           for (int i=0;i<w*h;i++) g_in[i]=77;
                       });

    /* ---- 第二段：外部向量 ---- */
    std::cout << "\n[2] 外部向量：流式实现 vs Python golden\n";
    const char *cases[] = { "gradient", "checker", "shapes", "ramp", "noise", "odd" };
    const int n_cases = (int)(sizeof(cases) / sizeof(cases[0]));
    int ran = 0, skipped = 0;
    for (int i = 0; i < n_cases; i++) {
        int r = test_vec(vec_dir, cases[i], thresh, gain);
        if (r < 0) { skipped++; continue; }
        ran++;
        if (r > 0) fails++;
    }
    if (ran == 0) {
        std::cout << "    （全部缺失，先执行 python sim/gen_vectors.py）\n";
    } else if (skipped > 0) {
        std::cout << "    （" << skipped << " 组向量缺失，已跳过）\n";
    }

    /* ---- 汇总 ---- */
    std::cout << "\n========================================================\n";
    if (fails == 0) {
        std::cout << "  TB PASSED\n";
        std::cout << "  （自一致性 8 组";
        if (ran > 0) std::cout << " + 外部向量 " << ran << " 组";
        std::cout << "，全部一致）\n";
        std::cout << "========================================================\n";
        return 0;
    }
    std::cout << "  TB FAILED —— " << fails << " 组不一致\n";
    std::cout << "========================================================\n";
    return 1;
}
