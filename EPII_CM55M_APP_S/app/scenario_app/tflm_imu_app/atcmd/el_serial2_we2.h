/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Seeed Technology Co.,Ltd
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#ifndef _EL_SERIAL2_WE2_H_
#define _EL_SERIAL2_WE2_H_

extern "C" {
#include <hx_drv_uart.h>
}

// #include "el_serial.h"
#include "el_types.h"
#include "el_ringbuffer.hpp"
#include "el_misc.h"

namespace edgelab {

class Serial2WE2 {
   public:
    el_transport_type_t type;
    Serial2WE2();
    ~Serial2WE2();

    el_err_code_t init();
    el_err_code_t deinit();

    char        echo(bool only_visible = true);
    char        get_char();
    std::size_t get_line(char* buffer, size_t size, const char delim = 0x0d);

    std::size_t read_bytes(char* buffer, size_t size);
    std::size_t send_bytes(const char* buffer, size_t size);

   protected:
    bool _is_present;

   private:
    DEV_UART* _console_uart;
};

}  // namespace edgelab

#endif
