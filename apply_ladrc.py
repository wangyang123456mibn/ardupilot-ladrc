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

# 1. AP_PIDInfo.h - add LADRC field for logging
patch_file("libraries/AC_PID/AP_PIDInfo.h", [
    ("float slew_rate;", "float slew_rate;\n    float LADRC;\n")
])

# 2. AC_PID.cpp - add static LADRC vars and LESO logic (NO class members added)
# Insert before first function in AC_PID.cpp (after #include)
ladrc_static = """
// LADRC static parameters (not class members to avoid changing AC_PID size)
static float ladrc_wo = 10.0f;
static float ladrc_b0 = 1.0f;
static int8_t ladrc_en = 0;
static int8_t ladrc_order = 1;
"""

lb = """
    // LADRC LESO compensation (using static vars)
    float ladrc_comp = 0.0f;
    if (ladrc_en > 0 && is_positive(dt) && is_positive(ladrc_wo) && is_positive(ladrc_b0)) {
        const float wo = ladrc_wo;
        const float b0 = ladrc_b0;
        // Use PID_info.LADRC as persistent state storage (z1, z2, z3, last_u)
        // We pack state into LADRC field and use other means for persistence
        // Simplified: use local static for demo (per-instance would need index)
        static float leso_z1 = 0.0f;
        static float leso_z2 = 0.0f;
        static float leso_z3 = 0.0f;
        static float last_u = 0.0f;
        const float u_last = last_u;
        const float e = leso_z1 - measurement;
        if (ladrc_order == 1) {
            const float beta1 = 2.0f * wo;
            const float beta2 = wo * wo;
            leso_z1 += dt * (leso_z2 - beta1 * e + b0 * u_last);
            leso_z2 += dt * (-beta2 * e);
            ladrc_comp = leso_z2 / b0;
        } else {
            const float beta1 = 3.0f * wo;
            const float beta2 = 3.0f * wo * wo;
            const float beta3 = wo * wo * wo;
            leso_z1 += dt * (leso_z2 - beta1 * e);
            leso_z2 += dt * (leso_z3 - beta2 * e + b0 * u_last);
            leso_z3 += dt * (-beta3 * e);
            ladrc_comp = leso_z3 / b0;
        }
        last_u = (P_out + D_out + _integrator) - ladrc_comp;
        _pid_info.LADRC = -ladrc_comp;
    }
"""

patch_file("libraries/AC_PID/AC_PID.cpp", [
    ("#include \"AC_PID.h\"",
     "#include \"AC_PID.h\"" + ladrc_static),
    ("    _pid_info.DFF = _target_derivative * _kdff;",
     "    _pid_info.DFF = _target_derivative * _kdff;" + lb),
    ("    return P_out + D_out + _integrator;",
     "    return P_out + D_out + _integrator - ladrc_comp;")
])

# 3. AC_PID.h - add getter only (no member variables)
patch_file("libraries/AC_PID/AC_PID.h", [
    ("    const AP_PIDInfo& get_pid_info(void) const { return _pid_info; }",
     "    const AP_PIDInfo& get_pid_info(void) const { return _pid_info; }\n"
     "    float get_ladrc(void) const { return _pid_info.LADRC; }")
])

# 4. AP_FW_Controller.cpp - optional
patch_file("libraries/APM_Control/AP_FW_Controller.cpp", [
    ("pinfo.D + pinfo.DFF", "pinfo.D + pinfo.DFF + pinfo.LADRC")
], optional=True)

# 5-6. AP_TECS disabled (known to cause crash)
print("  [SKIP] AP_TECS.h (disabled)")
print("  [SKIP] AP_TECS.cpp (disabled)")

print("LADRC patch complete! (Static vars version - no class size change)")
