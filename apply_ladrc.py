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

# ULTIMATE MINIMAL TEST: Only add one float field to AP_PIDInfo struct
# This does NOT change any class size, does NOT add parameters, does NOT add code
patch_file("libraries/AC_PID/AP_PIDInfo.h", [
    ("float slew_rate;", "float slew_rate;\n    float LADRC;")
])

# Everything else disabled
print("  [SKIP] AC_PID.h (testing minimal change)")
print("  [SKIP] AC_PID.cpp (testing minimal change)")
print("  [SKIP] AP_FW_Controller.cpp (disabled)")
print("  [SKIP] AP_TECS.h (disabled)")
print("  [SKIP] AP_TECS.cpp (disabled)")

print("LADRC patch complete! (Ultra-minimal test - only AP_PIDInfo.h)")
