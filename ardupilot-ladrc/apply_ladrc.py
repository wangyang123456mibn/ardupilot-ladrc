import os
BASE = os.getcwd()

# 1. AP_PIDInfo.h - add LADRC field
with open(os.path.join(BASE, "libraries/AC_PID/AP_PIDInfo.h"), "r") as f:
    lines = f.readlines()
with open(os.path.join(BASE, "libraries/AC_PID/AP_PIDInfo.h"), "w") as f:
    for line in lines:
        f.write(line)
        if "float slew_rate;" in line:
            f.write("    float LADRC;      // LADRC disturbance compensation term\n")
print("  [OK] AP_PIDInfo.h")

# 2. AC_PID.h - add LADRC member variables and getter
with open(os.path.join(BASE, "libraries/AC_PID/AC_PID.h"), "r") as f:
    content = f.read()

# Insert LADRC variables before SlewLimiter member
content = content.replace(
    "    SlewLimiter _slew_limiter{_slew_rate_max, _slew_rate_tau};",
    "    // LADRC parameters\n    AP_Float _ladrc_wo;\n    AP_Float _ladrc_b0;\n    AP_Int8  _ladrc_en;\n    AP_Int8  _ladrc_order;\n    float _leso_z1;\n    float _leso_z2;\n    float _leso_z3;\n    float _last_u_ladrc;\n\n    SlewLimiter _slew_limiter{_slew_rate_max, _slew_rate_tau};"
)

# Add LADRC getter after get_pid_info
content = content.replace(
    "    const AP_PIDInfo& get_pid_info(void) const { return _pid_info; }",
    "    const AP_PIDInfo& get_pid_info(void) const { return _pid_info; }\n    float get_ladrc(void) const { return _pid_info.LADRC; }"
)

with open(os.path.join(BASE, "libraries/AC_PID/AC_PID.h"), "w") as f:
    f.write(content)
print("  [OK] AC_PID.h")

# 3. AC_PID.cpp - add parameters, init, and LESO block
with open(os.path.join(BASE, "libraries/AC_PID/AC_PID.cpp"), "r") as f:
    content = f.read()

# Add AP_GROUPINFO parameters before AP_GROUPEND
content = content.replace(
    "    AP_GROUPEND",
    "    AP_GROUPINFO(\"LADRC_WO\",  17, AC_PID, _ladrc_wo,  0),\n    AP_GROUPINFO(\"LADRC_B0\",  18, AC_PID, _ladrc_b0,  1.0f),\n    AP_GROUPINFO(\"LADRC_EN\",  19, AC_PID, _ladrc_en,  0),\n    AP_GROUPINFO(\"LADRC_ORD\", 20, AC_PID, _ladrc_order, 1),\n    AP_GROUPEND"
)

# Add init in constructor - after _pid_info reset
content = content.replace(
    "    memset(&_pid_info, 0, sizeof(_pid_info));",
    "    _leso_z1 = 0.0f;\n    _leso_z2 = 0.0f;\n    _leso_z3 = 0.0f;\n    _last_u_ladrc = 0.0f;\n    memset(&_pid_info, 0, sizeof(_pid_info));"
)

# Add LESO block before return statement
if "_ladrc_wo.get()" not in content:
    lb = """
    // LADRC LESO compensation
    float ladrc_comp = 0.0f;
    if (_ladrc_en > 0 && is_positive(dt) && is_positive(_ladrc_wo) && is_positive(_ladrc_b0)) {
        const float wo = _ladrc_wo.get();
        const float b0 = _ladrc_b0.get();
        const float u_last = _last_u_ladrc;
        const float e = _leso_z1 - measurement;
        if (_ladrc_order == 1) {
            const float beta1 = 2.0f * wo;
            const float beta2 = wo * wo;
            _leso_z1 += dt * (_leso_z2 - beta1 * e + b0 * u_last);
            _leso_z2 += dt * (-beta2 * e);
            ladrc_comp = _leso_z2 / b0;
        } else {
            const float beta1 = 3.0f * wo;
            const float beta2 = 3.0f * wo * wo;
            const float beta3 = wo * wo * wo;
            _leso_z1 += dt * (_leso_z2 - beta1 * e);
            _leso_z2 += dt * (_leso_z3 - beta2 * e + b0 * u_last);
            _leso_z3 += dt * (-beta3 * e);
            ladrc_comp = _leso_z3 / b0;
        }
    }
    _pid_info.LADRC = -ladrc_comp;
    _last_u_ladrc = (P_out + D_out + I_out) - ladrc_comp;
"""
    content = content.replace(
        "    _pid_info.DFF = _target_derivative * _kdff;",
        "    _pid_info.DFF = _target_derivative * _kdff;" + lb
    )
    content = content.replace(
        "    return P_out + D_out + I_out;",
        "    return P_out + D_out + I_out - ladrc_comp;"
    )

with open(os.path.join(BASE, "libraries/AC_PID/AC_PID.cpp"), "w") as f:
    f.write(content)
print("  [OK] AC_PID.cpp")

# 4. AP_FW_Controller.cpp - add LADRC to output sum
with open(os.path.join(BASE, "libraries/APM_Control/AP_FW_Controller.cpp"), "r") as f:
    content = f.read()
content = content.replace("pinfo.D + pinfo.DFF", "pinfo.D + pinfo.DFF + pinfo.LADRC")
with open(os.path.join(BASE, "libraries/APM_Control/AP_FW_Controller.cpp"), "w") as f:
    f.write(content)
print("  [OK] AP_FW_Controller.cpp")

# 5. AP_TECS.h - add LADRC height control variables
with open(os.path.join(BASE, "libraries/AP_TECS/AP_TECS.h"), "r") as f:
    content = f.read()
ti = "    // LADRC height control\n    AP_Float _ladrc_hgt_wo;\n    AP_Float _ladrc_hgt_b0;\n    AP_Int8  _ladrc_hgt_en;\n    float _hgt_leso_z1;\n    float _hgt_leso_z2;\n    float _hgt_leso_z3;\n    float _last_pitch_dem_ladrc;\n"
content = content.replace("    void _update_pitch_limits", ti + "    void _update_pitch_limits")
with open(os.path.join(BASE, "libraries/AP_TECS/AP_TECS.h"), "w") as f:
    f.write(content)
print("  [OK] AP_TECS.h")

# 6. AP_TECS.cpp - add LADRC height control logic
with open(os.path.join(BASE, "libraries/AP_TECS/AP_TECS.cpp"), "r") as f:
    content = f.read()
content = content.replace("    AP_GROUPINFO(\"FLARE_HGT\"","    AP_GROUPINFO(\"HGT_WO\", 32, AP_TECS, _ladrc_hgt_wo, 0),\n    AP_GROUPINFO(\"HGT_B0\", 33, AP_TECS, _ladrc_hgt_b0, 5.0f),\n    AP_GROUPINFO(\"HGT_EN\", 34, AP_TECS, _ladrc_hgt_en, 0),\n    AP_GROUPINFO(\"FLARE_HGT\"")
content = content.replace("    _pitch_dem_unc = 0.0f;","    _hgt_leso_z1 = 0.0f;\n    _hgt_leso_z2 = 0.0f;\n    _hgt_leso_z3 = 0.0f;\n    _last_pitch_dem_ladrc = 0.0f;\n    _pitch_dem_unc = 0.0f;")
hl = "    if (_ladrc_hgt_en > 0 && is_positive(_ladrc_hgt_wo) && is_positive(_ladrc_hgt_b0) && is_positive(_DT)) {\n        const float wo  = _ladrc_hgt_wo.get();\n        const float b0  = _ladrc_hgt_b0.get();\n        const float beta1 = 3.0f * wo;\n        const float beta2 = 3.0f * wo * wo;\n        const float beta3 = wo * wo * wo;\n        const float e = _hgt_leso_z1 - _height;\n        _hgt_leso_z1 += _DT * (_hgt_leso_z2 - beta1 * e);\n        _hgt_leso_z2 += _DT * (_hgt_leso_z3 - beta2 * e + b0 * _last_pitch_dem_ladrc);\n        _hgt_leso_z3 += _DT * (-beta3 * e);\n        _pitch_dem_unc -= _hgt_leso_z3 / b0;\n    }\n    _pitch_dem = constrain_float(_pitch_dem_unc, _PITCHminf, _PITCHmaxf);\n    _last_pitch_dem_ladrc = _pitch_dem;\n"
content = content.replace("    _pitch_dem = constrain_float(_pitch_dem_unc, _PITCHminf, _PITCHmaxf);", hl)
with open(os.path.join(BASE, "libraries/AP_TECS/AP_TECS.cpp"), "w") as f:
    f.write(content)
print("  [OK] AP_TECS.cpp")
print("LADRC patch complete!")
