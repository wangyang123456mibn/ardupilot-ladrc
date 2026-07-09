import os

BASE = os.getcwd()

def patch_file(filepath, replacements, optional=False):
    fullpath = os.path.join(BASE, filepath)
    if not os.path.exists(fullpath):
        if optional:
            print(f"  [SKIP] {filepath} (not found, optional)")
            return True
        print(f"  [FAIL] {filepath} (not found)")
        return False

    with open(fullpath, "r") as f:
        content = f.read()

    original = content
    for old, new in replacements:
        if old in content:
            content = content.replace(old, new)
        else:
            print(f"  [WARN] {filepath}: pattern not found: {old[:50]}...")

    if content == original:
        print(f"  [OK] {filepath} (no changes needed)")
    else:
        with open(fullpath, "w") as f:
            f.write(content)
        print(f"  [OK] {filepath}")
    return True

# 1. AP_PIDInfo.h - add LADRC field
patch_file("libraries/AC_PID/AP_PIDInfo.h", [
    ("float slew_rate;", "float slew_rate;\n    float LADRC;      // LADRC disturbance compensation term\n")
])

# 2. AC_PID.h - add LADRC member variables and getter
patch_file("libraries/AC_PID/AC_PID.h", [
    ("    SlewLimiter _slew_limiter{_slew_rate_max, _slew_rate_tau};",
     "    // LADRC parameters\n"
     "    AP_Float _ladrc_wo;\n"
     "    AP_Float _ladrc_b0;\n"
     "    AP_Int8  _ladrc_en;\n"
     "    AP_Int8  _ladrc_order;\n"
     "    float _leso_z1;\n"
     "    float _leso_z2;\n"
     "    float _leso_z3;\n"
     "    float _last_u_ladrc;\n\n"
     "    SlewLimiter _slew_limiter{_slew_rate_max, _slew_rate_tau};"),
    ("    const AP_PIDInfo& get_pid_info(void) const { return _pid_info; }",
     "    const AP_PIDInfo& get_pid_info(void) const { return _pid_info; }\n"
     "    float get_ladrc(void) const { return _pid_info.LADRC; }")
])

# 3. AC_PID.cpp - add parameters, init, and LESO block
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

patch_file("libraries/AC_PID/AC_PID.cpp", [
    ("    AP_GROUPEND",
     "    AP_GROUPINFO(\"LADRC_WO\",  17, AC_PID, _ladrc_wo,  0),\n"
     "    AP_GROUPINFO(\"LADRC_B0\",  18, AC_PID, _ladrc_b0,  1.0f),\n"
     "    AP_GROUPINFO(\"LADRC_EN\",  19, AC_PID, _ladrc_en,  0),\n"
     "    AP_GROUPINFO(\"LADRC_ORD\", 20, AC_PID, _ladrc_order, 1),\n"
     "    AP_GROUPEND"),
    ("    memset(&_pid_info, 0, sizeof(_pid_info));",
     "    _leso_z1 = 0.0f;\n"
     "    _leso_z2 = 0.0f;\n"
     "    _leso_z3 = 0.0f;\n"
     "    _last_u_ladrc = 0.0f;\n"
     "    memset(&_pid_info, 0, sizeof(_pid_info));"),
    ("    _pid_info.DFF = _target_derivative * _kdff;",
     "    _pid_info.DFF = _target_derivative * _kdff;" + lb),
    ("    return P_out + D_out + I_out;",
     "    return P_out + D_out + I_out - ladrc_comp;")
])

# 4. AP_FW_Controller.cpp - add LADRC to output sum (optional, may not exist on stable)
patch_file("libraries/APM_Control/AP_FW_Controller.cpp", [
    ("pinfo.D + pinfo.DFF", "pinfo.D + pinfo.DFF + pinfo.LADRC")
], optional=True)

# 5. AP_TECS.h - add LADRC height control variables
patch_file("libraries/AP_TECS/AP_TECS.h", [
    ("    void _update_pitch_limits",
     "    // LADRC height control\n"
     "    AP_Float _ladrc_hgt_wo;\n"
     "    AP_Float _ladrc_hgt_b0;\n"
     "    AP_Int8  _ladrc_hgt_en;\n"
     "    float _hgt_leso_z1;\n"
     "    float _hgt_leso_z2;\n"
     "    float _hgt_leso_z3;\n"
     "    float _last_pitch_dem_ladrc;\n\n"
     "    void _update_pitch_limits")
])

# 6. AP_TECS.cpp - add LADRC height control logic
hl = """    if (_ladrc_hgt_en > 0 && is_positive(_ladrc_hgt_wo) && is_positive(_ladrc_hgt_b0) && is_positive(_DT)) {
        const float wo  = _ladrc_hgt_wo.get();
        const float b0  = _ladrc_hgt_b0.get();
        const float beta1 = 3.0f * wo;
        const float beta2 = 3.0f * wo * wo;
        const float beta3 = wo * wo * wo;
        const float e = _hgt_leso_z1 - _height;
        _hgt_leso_z1 += _DT * (_hgt_leso_z2 - beta1 * e);
        _hgt_leso_z2 += _DT * (_hgt_leso_z3 - beta2 * e + b0 * _last_pitch_dem_ladrc);
        _hgt_leso_z3 += _DT * (-beta3 * e);
        _pitch_dem_unc -= _hgt_leso_z3 / b0;
    }
    _pitch_dem = constrain_float(_pitch_dem_unc, _PITCHminf, _PITCHmaxf);
    _last_pitch_dem_ladrc = _pitch_dem;
"""

patch_file("libraries/AP_TECS/AP_TECS.cpp", [
    ("    AP_GROUPINFO(\"FLARE_HGT\"",
     "    AP_GROUPINFO(\"HGT_WO\", 32, AP_TECS, _ladrc_hgt_wo, 0),\n"
     "    AP_GROUPINFO(\"HGT_B0\", 33, AP_TECS, _ladrc_hgt_b0, 5.0f),\n"
     "    AP_GROUPINFO(\"HGT_EN\", 34, AP_TECS, _ladrc_hgt_en, 0),\n"
     "    AP_GROUPINFO(\"FLARE_HGT\""),
    ("    _pitch_dem_unc = 0.0f;",
     "    _hgt_leso_z1 = 0.0f;\n"
     "    _hgt_leso_z2 = 0.0f;\n"
     "    _hgt_leso_z3 = 0.0f;\n"
     "    _last_pitch_dem_ladrc = 0.0f;\n"
     "    _pitch_dem_unc = 0.0f;"),
    ("    _pitch_dem = constrain_float(_pitch_dem_unc, _PITCHminf, _PITCHmaxf);",
     hl)
])

print("LADRC patch complete!")
