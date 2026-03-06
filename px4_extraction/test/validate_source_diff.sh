#!/bin/bash
# =============================================================================
# validate_source_diff.sh — L1 Source Code Diff Audit
#
# Compares extracted px4_extraction/ files against original PX4 sources.
# Categorizes each file pair as:
#   IDENTICAL  — zero diff (verbatim copy)
#   MINIMAL    — only comments, includes, or middleware stubs changed
#   ADAPTED    — logic-equivalent refactor (e.g. param system replaced)
#   DIVERGENT  — unexpected logic difference (needs review)
# =============================================================================

set -uo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

PASS=0
WARN=0
FAIL=0

classify() {
    local orig="$ROOT/$1"
    local extr="$ROOT/$2"
    local max_comment_diff="${3:-0}"  # max allowed comment/include-only diff lines
    local max_logic_diff="${4:-0}"    # max allowed logic diff lines

    if [ ! -f "$orig" ]; then
        printf "${CYAN}SKIP${NC}    %-70s (original not found)\n" "$1"
        return
    fi
    if [ ! -f "$extr" ]; then
        printf "${CYAN}SKIP${NC}    %-70s (extracted not found)\n" "$2"
        return
    fi

    local total_diff
    total_diff=$(diff -u "$orig" "$extr" 2>/dev/null | grep '^[+-]' | grep -cv '^[+-][+-][+-]' || true)

    if [ "$total_diff" -eq 0 ]; then
        printf "${GREEN}IDENTICAL${NC} %-65s (0 diff lines)\n" "$2"
        ((PASS++))
        return
    fi

    # Count logic-only diffs (exclude comments, includes, param macros, ModuleParams)
    # Per AGENTS.md Strict Coding Directives, these removals are expected:
    #   - uORB, ModuleParams, DEFINE_PARAMETERS, param_*, ParamInt (Rule 1 & 2)
    #   - PX4 logging, hrt timers, work queues (Rule 2)
    #   - friend class (middleware coupling), ///< inline doxygen (cosmetic)
    #   - _param_mc_airmode / updateParameters (param system → plain member)
    local logic_diff
    logic_diff=$(diff -u "$orig" "$extr" 2>/dev/null \
        | grep '^[+-]' | grep -v '^[+-][+-][+-]' \
        | grep -v '^\s*[+-]\s*//' \
        | grep -v '^\s*[+-]\s*\*' \
        | grep -v '#include' \
        | grep -v 'ModuleParams' \
        | grep -v 'DEFINE_PARAMETERS' \
        | grep -v 'ParamInt' \
        | grep -v 'param_' \
        | grep -v 'uORB' \
        | grep -v 'control_allocator_status_s' \
        | grep -v 'px4_work_queue' \
        | grep -v 'PX4_WARN\|PX4_ERR\|PX4_INFO' \
        | grep -v 'hrt_absolute_time\|hrt_abstime' \
        | grep -v '@file\|@brief\|@param\|@return\|@author\|@see\|@note' \
        | grep -v '///<' \
        | grep -v 'friend class' \
        | grep -v '_param_mc_airmode' \
        | grep -v 'updateParameters' \
        | grep -v '^\s*[+-]\s*$' \
        | grep -cv '^\s*[+-]\s*\*/' || true)

    if [ "$logic_diff" -le "$max_logic_diff" ]; then
        printf "${GREEN}MINIMAL${NC}  %-66s (%d total, %d logic diffs)\n" "$2" "$total_diff" "$logic_diff"
        ((PASS++))
    elif [ "$logic_diff" -le $((max_logic_diff + 10)) ]; then
        printf "${YELLOW}ADAPTED${NC}  %-66s (%d total, %d logic diffs)\n" "$2" "$total_diff" "$logic_diff"
        ((WARN++))
    else
        printf "${RED}DIVERGENT${NC} %-65s (%d total, %d logic diffs)\n" "$2" "$total_diff" "$logic_diff"
        ((FAIL++))
    fi
}

echo "============================================================================="
echo "  L1 Source Code Diff Audit: px4_extraction vs PX4 original"
echo "  PX4 branch: $(cd "$ROOT" && git branch --show-current 2>/dev/null || echo 'unknown')"
echo "============================================================================="
echo ""

echo "--- Phase 1: Attitude Control ---"
classify \
    "src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp" \
    "px4_extraction/attitude_control/AttitudeControl.cpp" 0 0
classify \
    "src/modules/mc_att_control/AttitudeControl/AttitudeControl.hpp" \
    "px4_extraction/attitude_control/AttitudeControl.hpp" 0 0
echo ""

echo "--- Phase 2: Rate Control ---"
classify \
    "src/lib/rate_control/rate_control.cpp" \
    "px4_extraction/rate_control/RateControl.cpp" 10 3
classify \
    "src/lib/rate_control/rate_control.hpp" \
    "px4_extraction/rate_control/RateControl.hpp" 15 5
echo ""

echo "--- Phase 3: Control Allocation ---"
classify \
    "src/lib/control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp" \
    "px4_extraction/control_allocator/ActuatorEffectiveness.hpp" 80 25
classify \
    "src/lib/control_allocation/actuator_effectiveness/ActuatorEffectiveness.cpp" \
    "px4_extraction/control_allocator/ActuatorEffectiveness.cpp" 20 5
classify \
    "src/lib/control_allocation/control_allocation/ControlAllocation.cpp" \
    "px4_extraction/control_allocator/ControlAllocation.cpp" 5 2
classify \
    "src/lib/control_allocation/control_allocation/ControlAllocation.hpp" \
    "px4_extraction/control_allocator/ControlAllocation.hpp" 100 15
classify \
    "src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp" \
    "px4_extraction/control_allocator/ControlAllocationPseudoInverse.cpp" 5 2
classify \
    "src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.hpp" \
    "px4_extraction/control_allocator/ControlAllocationPseudoInverse.hpp" 10 2
classify \
    "src/lib/control_allocation/control_allocation/ControlAllocationSequentialDesaturation.cpp" \
    "px4_extraction/control_allocator/ControlAllocationSequentialDesaturation.cpp" 15 8
classify \
    "src/lib/control_allocation/control_allocation/ControlAllocationSequentialDesaturation.hpp" \
    "px4_extraction/control_allocator/ControlAllocationSequentialDesaturation.hpp" 80 15
echo ""

echo "--- Math Libraries ---"
classify \
    "src/lib/mathlib/math/filter/AlphaFilter.hpp" \
    "px4_extraction/lib/mathlib/math/filter/AlphaFilter.hpp" 0 0
classify \
    "src/lib/mathlib/math/Functions.hpp" \
    "px4_extraction/lib/mathlib/math/Functions.hpp" 0 0
classify \
    "src/lib/mathlib/math/Limits.hpp" \
    "px4_extraction/lib/mathlib/math/Limits.hpp" 0 0
echo ""

echo "============================================================================="
printf "  Results: ${GREEN}%d PASS${NC}  ${YELLOW}%d ADAPTED${NC}  ${RED}%d DIVERGENT${NC}\n" "$PASS" "$WARN" "$FAIL"
echo "============================================================================="

if [ "$FAIL" -gt 0 ]; then
    echo ""
    echo "DIVERGENT files need manual review — logic may have changed."
    exit 1
fi

exit 0
