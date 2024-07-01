/*
 * atcmd_server.h
 *
 *  Created on: 2024/06/12
 *      Author: Himax
 */

#include "atcmd_server.h"
#include "el_serial2_we2.h"
#include "cvapp_nycu_z_axsis.h"
#include "common_config.h"


static std::vector<repl_cmd_t> _cmd_list;
edgelab::Serial2WE2 serial2{};

char* g_cmd_buf = reinterpret_cast<char*>(new unsigned char[SSCMA_CMD_MAX_LENGTH + 1]);
IMURingBuffer g_imu_ringBuffer;
model_arg_t g_model_arg;
model_addr_t g_model_addr;
inference_result_t g_nn_result;

void atcmd_server_init() {

	serial2.init();
    serial2.type = EL_TRANSPORT_UART;

    register_commands();
}


void register_commands() {
    std::string cmd;

    el_printf("[ATCMD Server] registering AT commands...\n");

    _cmd_list.emplace_back("HELP", "Display help information", "", [](const std::string& argv) -> el_err_code_t {
        return list_command(argv);
    });

    _cmd_list.emplace_back("NAME", "Get device name", "", [](const std::string& argv) -> el_err_code_t {
        return get_device_name(argv);
    });

    _cmd_list.emplace_back("VERSION", "List firmware version", "", [](const std::string& argv) -> el_err_code_t {
        return get_fw_version(argv);
    });

    _cmd_list.emplace_back("STATUS", "Firmware status", "", [](const std::string& argv) -> el_err_code_t {
        return get_fw_status(argv);
    });

    _cmd_list.emplace_back("FACTORY", "Factory settings restored", "", [](const std::string& argv) -> el_err_code_t {
        return set_factory(argv);
    });

    _cmd_list.emplace_back("SETNN", "Set NN parameters", "", [](const std::string& argv) -> el_err_code_t {
        return set_nn_param(argv);
    });

    _cmd_list.emplace_back("GETNN", "Get NN parameters", "", [](const std::string& argv) -> el_err_code_t {
        return get_nn_param(argv);
    });

    _cmd_list.emplace_back("DFU", "Firmware Upgrade", "", [](const std::string& argv) -> el_err_code_t {
        return upgrade_fw(argv);
    });

    _cmd_list.emplace_back("LOADNN", "Load Model", "", [](const std::string& argv) -> el_err_code_t {
        return upload_model(argv);
    });

    _cmd_list.emplace_back("SENDIMU", "Send IMU data", "", [](const std::string& argv) -> el_err_code_t {
        return read_imu_data(argv);
    });

    _cmd_list.emplace_back("NNRUN", "Run NN inference", "", [](const std::string& argv) -> el_err_code_t {
        return run_nn_inference(argv);
    });

    _cmd_list.emplace_back("GETRESULT", "Get NN inference result", "", [](const std::string& argv) -> el_err_code_t {
        return get_result(argv);
    });

    _cmd_list.emplace_back("RESET", "System reset", "", [](const std::string& argv) -> el_err_code_t {
        return we2_reset(argv);
    });

    std::cout << "Commands in the container:\r\n";
    for (const auto& cmd : _cmd_list) {
        std::cout << "Command: " << cmd.cmd << ", Description: " << cmd.desc << "\r\n";
    }

    // Output ATCMD Server Message to UART1
    std::string ss = "\r\n[ATCMD Server] start ...\r\n";
    serial2.send_bytes(ss.c_str(), ss.size());
}


el_err_code_t get_cmd(std::string& cmd_line) {
    memset(g_cmd_buf, 0, SSCMA_CMD_MAX_LENGTH + 1);

	if ( serial2.get_line(g_cmd_buf, SSCMA_CMD_MAX_LENGTH) )
	{
        cmd_line = std::move(g_cmd_buf);
        auto it = std::remove_if(cmd_line.begin(), cmd_line.end(), [](char c) { return !std::isprint(c); });
        cmd_line.erase(it, cmd_line.end());
        return EL_OK;
    }
    else
        return EL_FAILED;
}


el_err_code_t exec_cmd(const std::string cmd) {
	el_err_code_t ret = EL_EINVAL;
	std::string   cmd_name;
	std::string   cmd_args;
    std::string   cmd_rpl;

	// find first '=' in AT command (<cmd_name>=<cmd_args>)
	size_t pos = cmd.find_first_of("=");
	if (pos != std::string::npos) {
		cmd_name = cmd.substr(0, pos);
		cmd_args = cmd.substr(pos + 1, cmd.size());
	} else
		cmd_name = cmd;

	std::transform(cmd_name.begin(), cmd_name.end(), cmd_name.begin(), ::toupper);

	// check if cmd_name is valid (starts with "AT+")
	if (cmd_name.rfind("AT+", 0) != 0) {
		//m_echo_cb(caller, EL_EINVAL, "Unknown command: ", cmd, "\n");
		std::cout << "Unknown command: " << cmd << std::endl;
		return EL_EINVAL;
	}
	cmd_name = cmd_name.substr(3, cmd_name.size());  // remove "AT+" command prefix

	// find command tag (everything after last '@'), then remove the tag to get the real command name
	size_t cmd_body_pos    = cmd_name.rfind('@');
	cmd_body_pos           = cmd_body_pos != std::string::npos ? cmd_body_pos + 1 : 0;
	std::string target_cmd = cmd_name.substr(cmd_body_pos, cmd_name.size());

    auto it = std::find_if(
        _cmd_list.begin(), _cmd_list.end(), [&](const repl_cmd_t& c) { return c.cmd.compare(target_cmd) == 0; });

    if (it != _cmd_list.end()) {
        it->cmd_cb(cmd);
    } else {
        cmd_rpl = "Command not found : " + target_cmd + "\r\n";
        serial2.send_bytes(cmd_rpl.c_str(), cmd_rpl.size());
        //std::cout << "Command not found: " << target_cmd << "\r\n";
    }

	return ret;
}


el_err_code_t list_command(const std::string& cmd) {
    std::ostringstream cmd_str;

    for (const auto& it : _cmd_list) {
        cmd_str << "AT+" << it.cmd << " : " << it.desc << "\r\n";
    }

    std::string ss = cmd_str.str();
    serial2.send_bytes(ss.c_str(), ss.size());

	return EL_OK;
}


el_err_code_t get_device_name(const std::string& cmd) {
    std::string ss = "Grove Vision AI V2\r\n";

    serial2.send_bytes(ss.c_str(), ss.size());
	return EL_OK;
}


el_err_code_t get_fw_version(const std::string& cmd) {
    std::ostringstream ss;
    ss << "Firmware version : " << std::hex << FW_MAJOR_VERSION << "." << std::hex << FW_MINOR_VERSION << "\r\n";
    std::string firmware_version_str = ss.str();

    serial2.send_bytes(firmware_version_str.c_str(), firmware_version_str.size());
	return EL_OK;
}


el_err_code_t get_fw_status(const std::string& cmd) {
    std::string ss = std::string("Firmware status : ") + "Running" + "\r\n";

    serial2.send_bytes(ss.c_str(), ss.size());
	return EL_OK;
}


el_err_code_t set_factory(const std::string& cmd) {
    std::string ss = "Factory settings restored\r\n";

    serial2.send_bytes(ss.c_str(), ss.size());
	return EL_OK;
}


el_err_code_t set_nn_param(const std::string& cmd) {
    std::string::size_type pos = cmd.find("SETNN=");
    std::string stripped_data = cmd.substr(pos + 6); // 去掉 "SETNN="
    stripped_data = removeExtraCommas(stripped_data); // 去掉多餘的連續逗號

    std::vector<int16_t> results;
    parseParam(stripped_data, results);

    g_model_arg.count = results.size();
    std::string ss = "SETNN=";
    for (size_t i = 0; i < results.size(); ++i) {
        if (i > 0) {
            ss += ",";
        }
        std::ostringstream oss;
        oss << std::hex << results[i];
        g_model_arg.arg[i] = results[i];
        ss += oss.str();
    }
    ss += "\r\n";
    serial2.send_bytes(ss.c_str(), ss.size());
	return EL_OK;
}


el_err_code_t get_nn_param(const std::string& cmd) {
    std::string ss = "GETNN=";
    for (int i = 0; i < g_model_arg.count; ++i) {
        if (i > 0) {
            ss += ",";
        }
        std::ostringstream oss;
        oss << std::hex << g_model_arg.arg[i];
        ss += oss.str();
    }
    ss += "\r\n";
    serial2.send_bytes(ss.c_str(), ss.size());
	return EL_OK;
}


el_err_code_t upgrade_fw(const std::string& cmd) {
    std::string ss = "Go to 2nd firmware upgarde\r\n";
    serial2.send_bytes(ss.c_str(), ss.size());

    app_enter_2nd_bl(1);
	return EL_OK;
}


el_err_code_t upload_model(const std::string& cmd) {
    std::string ss = "LOADNN=";
    std::string::size_type pos = cmd.find(ss);
    std::string stripped_data = cmd.substr(pos + 7); // 去掉 "LOADNN="

    if ( parseModel(stripped_data, g_model_addr) == false )
    {
        ss = "LOADNN=Syntax Error!\r\n";
        serial2.send_bytes(ss.c_str(), ss.size());
        return EL_OK;
    }

    std::ostringstream oss;
    oss << std::hex << g_model_addr.id << "," << g_model_addr.name << "," << std::hex << g_model_addr.addr;
    ss += oss.str();
    ss += "\r\n";
    serial2.send_bytes(ss.c_str(), ss.size());
    app_enter_2nd_bl(1);
	return EL_OK;
}


el_err_code_t read_imu_data(const std::string& cmd) {
    std::string size_str;
    std::string ss = cmd + "\r\n";

    // serial2.send_bytes(ss.c_str(), ss.size());

    parseIMUdata(cmd, g_imu_ringBuffer);

    size_str = std::to_string(g_imu_ringBuffer.getSize());
    ss = "SENDIMU=" + size_str + "\r\n";
    serial2.send_bytes(ss.c_str(), ss.size());

	return EL_OK;
}


el_err_code_t run_nn_inference(const std::string& cmd) {
    std::string size_str;

    std::string ss = cmd + "\r\n";
    serial2.send_bytes(ss.c_str(), ss.size());

    std::cout << "g_imu_ringBuffer size = " << g_imu_ringBuffer.getSize() << "\r\n";
    cv_nycu_z_axsis_run();

	return EL_OK;
}


el_err_code_t get_result(const std::string& cmd) {
    std::string ss = "GETRESULT=";
    std::ostringstream oss;
    oss << static_cast<int>(g_nn_result.output[0]) << "," << static_cast<int>(g_nn_result.output[1]) << "," << static_cast<int>(g_nn_result.output[2]);
    ss += oss.str();
    ss += "\r\n";
    printf("output[0] = 0x%x, output[1] = 0x%x, output[2] = 0x%x\r\n", g_nn_result.output[0], g_nn_result.output[1], g_nn_result.output[2]);
    serial2.send_bytes(ss.c_str(), ss.size());
	return EL_OK;
}


el_err_code_t we2_reset(const std::string& cmd) {
    std::string ss = "System Reset!\r\n";
    serial2.send_bytes(ss.c_str(), ss.size());

    app_enter_2nd_bl(0);
	return EL_OK;
}


// 去掉多餘的連續逗號
std::string removeExtraCommas(const std::string& data) {
    std::string result;
    bool lastCharWasComma = false;

    for (char c : data) {
        if (c != ',' || !lastCharWasComma) {
            result += c;
        }
        lastCharWasComma = (c == ',');
    }

    return result;
}


bool isValidHex(const std::string& str) {
    for (char c : str) {
        if (!std::isxdigit(c)) {
            return false;
        }
    }
    return true;
}


// 解析字串並將其存放到環形緩衝區中
void parseIMUdata(const std::string& data, IMURingBuffer& ringBuffer) {
    std::string::size_type pos = data.find("SENDIMU=");
    if (pos != std::string::npos) {
        std::string stripped_data = data.substr(pos + 8); // 去掉 "SENDIMU="
        stripped_data = removeExtraCommas(stripped_data); // 去掉多餘的連續逗號

        std::istringstream ss(stripped_data);
        std::string token;

        while (std::getline(ss, token, ',')) {
            if (isValidHex(token)) {
                int16_t value = static_cast<int16_t>(std::stoi(token, nullptr, 16)); // 將十六進位制字串轉換為int16_t
                if (!ringBuffer.add(value)) {
                    break;
                }
            } else {
                std::cerr << "Invalid hex string: " << token << std::endl;
            }
        }
    } else {
        std::cerr << "Invalid data format: 'SENDIMU=' not found" << std::endl;
    }
}


int parseParam(const std::string& stripped_data, std::vector<int16_t>& results) {
    std::string clean_data = removeExtraCommas(stripped_data); // 去掉多余的连续逗号

    std::istringstream ss(clean_data);
    std::string token;
    int count = 0;

    while (std::getline(ss, token, ',')) {
        if (isValidHex(token)) {
            int16_t value = static_cast<int16_t>(std::stoi(token, nullptr, 16)); // 将十六进制字符串转换为int16_t
            results.push_back(value);
            ++count;
        } else {
            std::cerr << "Invalid hex string: " << token << std::endl;
        }
    }
    return count;
}


// 將字串中的內容解析為指定的格式並存儲到 model_addr_t 結構中
bool parseModel(const std::string& data, model_addr_t& model) {
    std::string cleanedData = removeExtraCommas(data);
    std::istringstream ss(cleanedData);
    std::string token;
    std::vector<std::string> tokens;

    while (std::getline(ss, token, ',')) {
        tokens.push_back(token);
    }

    if (tokens.size() >= 3) {
        if (isValidHex(tokens[0])) {
            model.id = static_cast<int16_t>(std::stoi(tokens[0], nullptr, 16));
        } else {
            std::cerr << "Invalid hex string: " << tokens[0] << std::endl;
            return false;
        }

        strncpy(model.name, tokens[1].c_str(), sizeof(model.name) - 1);
        model.name[sizeof(model.name) - 1] = '\0';  // 保證字串以空字元結尾

        if (isValidHex(tokens[2])) {
            model.addr = std::stoul(tokens[2], nullptr, 16);
        } else {
            std::cerr << "Invalid hex string: " << tokens[2] << std::endl;
            return false;
        }

        return true;
    } else {
        std::cerr << "Invalid data format, expected at least 3 tokens." << std::endl;
        return false;
    }
}
