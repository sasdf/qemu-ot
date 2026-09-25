import re

with open("hw/opentitan/ot_edn.c", "r") as f:
    text = f.read()

# Replace the fatal error in ot_edn_check_command_ready with just tracing and returning false.
new_text = re.sub(
    r'(xtrace_ot_edn_error\(c->appid, "Another command is already scheduled"\);\s*)s->regs\[R_ERR_CODE\] \|= R_ERR_CODE_EDN_MAIN_SM_ERR_MASK;\s*ot_edn_change_state\(s, EDN_ERROR\);\s*return false;',
    r'\1return false;',
    text,
    flags=re.MULTILINE
)

with open("hw/opentitan/ot_edn.c", "w") as f:
    f.write(new_text)

