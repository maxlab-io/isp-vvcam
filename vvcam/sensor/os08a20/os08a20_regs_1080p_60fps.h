/****************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 VeriSilicon Holdings Co., Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 *****************************************************************************
 *
 * The GPL License (GPL)
 *
 * Copyright (c) 2020 VeriSilicon Holdings Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program;
 *
 *****************************************************************************
 *
 * Note: This software is released under dual MIT and GPL licenses. A
 * recipient may use this file under the terms of either the MIT license or
 * GPL License. If you wish to use only one license not the other, you can
 * indicate your decision by deleting one of the above license notices in your
 * version of this file.
 *
 *****************************************************************************/
#ifndef _VVCAM_OS08a20_REGS_1080P_H_
#define _VVCAM_OS08a20_REGS_1080P_H_

#include "os08a20_mipi_v3.h"

static struct reg_value os08a20_init_setting_1080p[] = {
	{0x0100, 0x00, 0, 0},
	{0x0103, 0x01, 0, 0},
	{0x0303, 0x01, 0, 0},
	{0x0305, 0x32, 0, 0},
	{0x0306, 0x00, 0, 0},
	{0x0308, 0x03, 0, 0},
	{0x0309, 0x04, 0, 0},
	{0x032a, 0x0b, 0, 0},
	{0x300f, 0x11, 0, 0},
	{0x3010, 0x01, 0, 0},
	{0x3011, 0x04, 0, 0},
	{0x3012, 0x41, 0, 0},
	{0x3016, 0xf0, 0, 0},
	{0x301e, 0x98, 0, 0},
	{0x3031, 0xa9, 0, 0},
	{0x3103, 0x92, 0, 0},
	{0x3104, 0x01, 0, 0},
	{0x3106, 0x10, 0, 0},
	{0x3400, 0x04, 0, 0},
	{0x3025, 0x03, 0, 0},
	{0x3425, 0x01, 0, 0},
	{0x3428, 0x01, 0, 0},
	{0x3406, 0x08, 0, 0},
	{0x3408, 0x03, 0, 0},
	{0x340c, 0xff, 0, 0},
	{0x340d, 0xff, 0, 0},
	{0x031e, 0x09, 0, 0},
	{0x3501, 0x04, 0, 0},
	{0x3502, 0x62, 0, 0},
	{0x3505, 0x83, 0, 0},
	{0x3508, 0x00, 0, 0},
	{0x3509, 0x80, 0, 0},
	{0x350a, 0x04, 0, 0},
	{0x350b, 0x00, 0, 0},
	{0x350c, 0x00, 0, 0},
	{0x350d, 0x80, 0, 0},
	{0x350e, 0x04, 0, 0},
	{0x350f, 0x00, 0, 0},
	{0x3600, 0x09, 0, 0},
	{0x3603, 0x2c, 0, 0},
	{0x3605, 0x50, 0, 0},
	{0x3609, 0xb5, 0, 0},
	{0x3610, 0x39, 0, 0},
	{0x360c, 0x01, 0, 0},
	{0x3628, 0xa4, 0, 0},
	{0x362d, 0x10, 0, 0},
	{0x3660, 0x43, 0, 0},
	{0x3661, 0x06, 0, 0},
	{0x3662, 0x00, 0, 0},
	{0x3663, 0x28, 0, 0},
	{0x3664, 0x0d, 0, 0},
	{0x366a, 0x38, 0, 0},
	{0x366b, 0xa0, 0, 0},
	{0x366d, 0x00, 0, 0},
	{0x366e, 0x00, 0, 0},
	{0x3680, 0x00, 0, 0},
	{0x36c0, 0x00, 0, 0},
	{0x3701, 0x02, 0, 0},
	{0x373b, 0x02, 0, 0},
	{0x373c, 0x02, 0, 0},
	{0x3736, 0x02, 0, 0},
	{0x3737, 0x02, 0, 0},
	{0x3705, 0x00, 0, 0},
	{0x3706, 0x39, 0, 0},
	{0x370a, 0x00, 0, 0},
	{0x370b, 0x98, 0, 0},
	{0x3709, 0x49, 0, 0},
	{0x3714, 0x22, 0, 0},
	{0x371c, 0x00, 0, 0},
	{0x371d, 0x08, 0, 0},
	{0x3740, 0x1b, 0, 0},
	{0x3741, 0x04, 0, 0},
	{0x375e, 0x0b, 0, 0},
	{0x3760, 0x10, 0, 0},
	{0x3776, 0x10, 0, 0},
	{0x3781, 0x02, 0, 0},
	{0x3782, 0x04, 0, 0},
	{0x3783, 0x02, 0, 0},
	{0x3784, 0x08, 0, 0},
	{0x3785, 0x08, 0, 0},
	{0x3788, 0x01, 0, 0},
	{0x3789, 0x01, 0, 0},
	{0x3797, 0x04, 0, 0},
	{0x3762, 0x11, 0, 0},
	{0x3800, 0x00, 0, 0},
	{0x3801, 0x00, 0, 0},
	{0x3802, 0x00, 0, 0},
	{0x3803, 0x0c, 0, 0},
	{0x3804, 0x0e, 0, 0},
	{0x3805, 0xff, 0, 0},
	{0x3806, 0x08, 0, 0},
	{0x3807, 0x6f, 0, 0},
	{0x3808, 0x07, 0, 0},
	{0x3809, 0x80, 0, 0},
	{0x380a, 0x04, 0, 0},
	{0x380b, 0x38, 0, 0},
	{0x380c, 0x0c, 0, 0},
	{0x380d, 0x04, 0, 0},
	{0x380e, 0x08, 0, 0},
	{0x380f, 0x58, 0, 0},
	{0x3813, 0x08, 0, 0},
	{0x3814, 0x03, 0, 0},
	{0x3815, 0x01, 0, 0},
	{0x3816, 0x03, 0, 0},
	{0x3817, 0x01, 0, 0},
	{0x381c, 0x00, 0, 0},
	{0x3820, 0x01, 0, 0},
	{0x3821, 0x05, 0, 0},
	{0x3823, 0x08, 0, 0},
	{0x3826, 0x00, 0, 0},
	{0x3827, 0x08, 0, 0},
	{0x382d, 0x08, 0, 0},
	{0x3832, 0x02, 0, 0},
	{0x3833, 0x00, 0, 0},
	{0x383c, 0x48, 0, 0},
	{0x383d, 0xff, 0, 0},
	{0x3d85, 0x0b, 0, 0},
	{0x3d84, 0x40, 0, 0},
	{0x3d8c, 0x63, 0, 0},
	{0x3d8d, 0xd7, 0, 0},
	{0x4000, 0xf8, 0, 0},
	{0x4001, 0x2b, 0, 0},
	{0x4004, 0x00, 0, 0},
	{0x4005, 0x40, 0, 0},
	{0x400a, 0x01, 0, 0},
	{0x400f, 0xa0, 0, 0},
	{0x4010, 0x12, 0, 0},
	{0x4018, 0x00, 0, 0},
	{0x4008, 0x02, 0, 0},
	{0x4009, 0x05, 0, 0},
	{0x401a, 0x58, 0, 0},
	{0x4050, 0x00, 0, 0},
	{0x4051, 0x01, 0, 0},
	{0x4028, 0x2f, 0, 0},
	{0x4052, 0x00, 0, 0},
	{0x4053, 0x80, 0, 0},
	{0x4054, 0x00, 0, 0},
	{0x4055, 0x80, 0, 0},
	{0x4056, 0x00, 0, 0},
	{0x4057, 0x80, 0, 0},
	{0x4058, 0x00, 0, 0},
	{0x4059, 0x80, 0, 0},
	{0x430b, 0xff, 0, 0},
	{0x430c, 0xff, 0, 0},
	{0x430d, 0x00, 0, 0},
	{0x430e, 0x00, 0, 0},
	{0x4501, 0x98, 0, 0},
	{0x4502, 0x00, 0, 0},
	{0x4643, 0x00, 0, 0},
	{0x4640, 0x01, 0, 0},
	{0x4641, 0x04, 0, 0},
	{0x4800, 0x64, 0, 0},
	{0x4809, 0x2b, 0, 0},
	{0x4813, 0x90, 0, 0},
	{0x4817, 0x04, 0, 0},
	{0x4833, 0x18, 0, 0},
	{0x4837, 0x14, 0, 0},
	{0x483b, 0x00, 0, 0},
	{0x484b, 0x03, 0, 0},
	{0x4850, 0x7c, 0, 0},
	{0x4852, 0x06, 0, 0},
	{0x4856, 0x58, 0, 0},
	{0x4857, 0xaa, 0, 0},
	{0x4862, 0x0a, 0, 0},
	{0x4869, 0x18, 0, 0},
	{0x486a, 0xaa, 0, 0},
	{0x486e, 0x03, 0, 0},
	{0x486f, 0x55, 0, 0},
	{0x4875, 0xf0, 0, 0},
	{0x5000, 0x89, 0, 0},
	{0x5001, 0x42, 0, 0},
	{0x5004, 0x40, 0, 0},
	{0x5005, 0x00, 0, 0},
	{0x5180, 0x00, 0, 0},
	{0x5181, 0x10, 0, 0},
	{0x580b, 0x03, 0, 0},
	{0x4d00, 0x03, 0, 0},
	{0x4d01, 0xc9, 0, 0},
	{0x4d02, 0xbc, 0, 0},
	{0x4d03, 0xc6, 0, 0},
	{0x4d04, 0x4a, 0, 0},
	{0x4d05, 0x25, 0, 0},
	{0x4700, 0x2b, 0, 0},
	{0x4e00, 0x2b, 0, 0},
	{0x0100, 0x01, 0, 0},
	{0x0100, 0x01, 0, 0},
	{0x0100, 0x01, 0, 0},
	{0x0100, 0x01, 0, 0},

};

#endif
