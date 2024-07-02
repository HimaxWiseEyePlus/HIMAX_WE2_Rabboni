/*
 * atcmd_server.h
 *
 *  Created on: 2024/06/12
 *      Author: Himax
 */

#ifndef _ATCMD_SERVER_H_
#define _ATCMD_SERVER_H_

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <algorithm>
#include <cstring>
#include <cctype>
#include <array>
#include <atomic>
#include <sstream>
#include <regex>
#include <stdexcept>

#include "el_types.h"
#include "el_ringbuffer.hpp"
#include "el_misc.h"
#include "el_serial2_we2.h"
#include "el_types.h"

extern "C" {
#include "hx_drv_gpio.h"
#include "hx_drv_swreg_aon.h"
#include "hx_drv_timer.h"
#include "hx_drv_pmu_export.h"
#include "powermode.h"
#include "el_2nd_bl.h"
}


#define     SSCMA_CMD_MAX_LENGTH	4906

typedef std::function<el_err_code_t(const std::string&)> repl_cmd_cb_t;

struct repl_cmd_t {
    template <typename Callable>
    repl_cmd_t(std::string cmd_, std::string desc_, std::string args_, Callable&& cmd_cb_)
        : cmd(std::move(cmd_)), desc(std::move(desc_)), args(std::move(args_)), cmd_cb(std::forward<Callable>(cmd_cb_)), _argc(0) {
        if (!this->args.empty()) {
            _argc = std::count(this->args.begin(), this->args.end(), ',') + 1;
        }
    }

    ~repl_cmd_t() = default;

    std::string   cmd;
    std::string   desc;
    std::string   args;
    repl_cmd_cb_t cmd_cb;

    uint8_t _argc;
};


class IMURingBuffer {
public:
    IMURingBuffer() : start_idx(0), end_idx(0), size(0) {}

    // 添加資料到緩衝區
    bool add(int16_t value) {
        if (isFull()) {
            std::cerr << "Buffer is full, cannot add data." << std::endl;
            return false;
        }
        buffer[end_idx] = value;
        end_idx = (end_idx + 1) % buffer.size();
        size++;
        return true;
    }

    // 從緩衝區讀取資料，指定讀取的筆數
    std::vector<int16_t> get(size_t num_items) {
        std::vector<int16_t> result;
        if (num_items > size) {
            num_items = size;  // 調整讀取的筆數不能超過當前緩衝區中的資料量
        }

        for (size_t i = 0; i < num_items; ++i) {
            result.push_back(buffer[start_idx]);
            start_idx = (start_idx + 1) % buffer.size();
            size--;
        }

        return result;
    }

    // 获取单个值
    int16_t getSingle() {
        if (isEmpty()) {
            return 0;
        }
        int16_t value = buffer[start_idx];
        start_idx = (start_idx + 1) % buffer.size();
        size--;
        return value;
    }

    // 檢查緩衝區是否已滿
    bool isFull() const {
        return size == buffer.size();
    }

    // 檢查緩衝區是否為空
    bool isEmpty() const {
        return size == 0;
    }

    // 取得目前緩衝區中的資料個數
    size_t getSize() const {
        return size;
    }

private:
    std::array<int16_t, 4096> buffer;
    std::atomic<size_t> start_idx;
    std::atomic<size_t> end_idx;
    std::atomic<size_t> size;
};

typedef struct model_addr_t {
    int16_t id;
    char name[25];
    uint32_t addr;
} model_addr_t;

typedef struct model_arg_t {
    int16_t count;
    int16_t arg[10];
} model_arg_t;


typedef struct inference_result_t {
    int8_t output[3];
} inference_result_t;


void atcmd_server_init();
void register_commands();
el_err_code_t get_cmd(std::string& cmd_line);
el_err_code_t exec_cmd(const std::string cmd);
el_err_code_t list_command(const std::string& cmd);
el_err_code_t get_device_name(const std::string& cmd);
el_err_code_t get_fw_version(const std::string& cmd);
el_err_code_t get_fw_status(const std::string& cmd);
el_err_code_t set_factory(const std::string& cmd);
el_err_code_t set_nn_param(const std::string& cmd);
el_err_code_t get_nn_param(const std::string& cmd);
el_err_code_t upgrade_fw(const std::string& cmd);
el_err_code_t upload_model(const std::string& cmd);
el_err_code_t read_imu_data(const std::string& cmd);
el_err_code_t run_nn_inference(const std::string& cmd);
el_err_code_t get_result(const std::string& cmd);
el_err_code_t we2_reset(const std::string& cmd);

std::string removeExtraCommas(const std::string& data);
bool isValidHex(const std::string& str);
void parseIMUdata(const std::string& data, IMURingBuffer& ringBuffer);
int parseParam(const std::string& stripped_data, std::vector<int16_t>& results);
bool parseModel(const std::string& data, model_addr_t& model);

#endif
