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

# ===== LADRC v2: Multi-instance independent LESO states =====

# 1. AP_PIDInfo.h - add LADRC field for logging
patch_file("libraries/AC_PID/AP_PIDInfo.h", [
    ("float slew_rate;", "float slew_rate;\n    float LADRC;\n")
])

# 2. AC_PID.cpp - add multi-instance LADRC with macro-configurable parameters
ladrc_header = """
// ========== LADRC CONFIGURATION ==========
// Set to 1 to enable LADRC, 0 to disable (pure PID)
#define LADRC_ENABLED 1

// LADRC parameters (adjust before compiling)
#define LADRC_WO      10.0f   // Observer bandwidth (rad/s)
#define LADRC_B0       1.0f   // Control channel gain
#define LADRC_ORDER    1      // 1=1st-order LESO, 2=2nd-order LESO

// Per-instance LESO state (max 8 PID controllers)
struct LADRC_State {
    const void* pid_ptr;
    float z1, z2, z3;
    float last_u;
};
static LADRC_State ladrc_states[8];
static uint8_t ladrc_num_states = 0;

static LADRC_State* get_ladrc_state(const void* pid) {
    for (uint8_t i = 0; i < ladrc_num_states; i++) {
        if (ladrc_states[i].pid_ptr == pid) return &ladrc_states[i];
    }
    if (ladrc_num_states < 8) {
        ladrc_states[ladrc_num_states].pid_ptr = pid;
        ladrc_states[ladrc_num_states].z1 = 0.0f;
        ladrc_states[ladrc_num_states].z2 = 0.0f;
        ladrc_states[ladrc_num_states].z3 = 0.0f;
        ladrc_states[ladrc_num_states].last_u = 0.0f;
        return &ladrc_states[ladrc_num_states++];
    }
    return nullptr;
}
// ========== LADRC END ==========
"""

lb = """
#if LADRC_ENABLED
    // LADRC LESO compensation (per-instance via this pointer)
    float ladrc_comp = 0.0f;
    if (is_positive(dt)) {
        LADRC_State* st = get_ladrc_state(this);
        if (st != nullptr) {
            const float wo = LADRC_WO;
            const float b0 = LADRC_B0;
            const float u_last = st->last_u;
            const float e = st->z1 - measurement;
            if (LADRC_ORDER == 1) {
                const float beta1 = 2.0f * wo;
                const float beta2 = wo * wo;
                st->z1 += dt * (st->z2 - beta1 * e + b0 * u_last);
                st->z2 += dt * (-beta2 * e);
                ladrc_comp = st->z2 / b0;
            } else {
                const float beta1 = 3.0f * wo;
                const float beta2 = 3.0f * wo * wo;
                const float beta3 = wo * wo * wo;
                st->z1 += dt * (st->z2 - beta1 * e);
                st->z2 += dt * (st->z3 - beta2 * e + b0 * u_last);
                st->z3 += dt * (-beta3 * e);
                ladrc_comp = st->z3 / b0;
            }
            _pid_info.LADRC = -ladrc_comp;
            st->last_u = (P_out + D_out + _integrator) - ladrc_comp;
        }
    }
#endif
"""

patch_file("libraries/AC_PID/AC_PID.cpp", [
    ("#include \"AC_PID.h\"",
     "#include \"AC_PID.h\"" + ladrc_header),
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

# 5-6. AP_TECS disabled (would need similar static-var refactoring)
print("  [SKIP] AP_TECS.h (disabled - needs separate refactoring)")
print("  [SKIP] AP_TECS.cpp (disabled - needs separate refactoring)")

print("LADRC patch v2 complete! (Multi-instance, macro-configurable)")

# Extract .hex files from .apj files (for STM32CubeProgrammer flashing)
def extract_hex():
    import json, glob
    for apj in glob.glob('build/*/bin/*.apj'):
        with open(apj) as f:
            data = json.load(f)
        hex_path = apj.replace('.apj', '.hex')
        with open(hex_path, 'w') as f:
            f.write(data['image'])
        print(f"Extracted: {hex_path}")
