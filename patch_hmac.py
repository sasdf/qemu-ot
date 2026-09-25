import re

with open("hw/opentitan/ot_hmac.c", "r") as f:
    text = f.read()

# We can replace the FIFO threshold logic to defer to a BH or just process less?
# Actually, if we just process only when HASH_PROCESS is written, does the ROM pass?
# Real HMAC hardware starts processing the block AS SOON AS 16 words are in the FIFO!
# "when the FIFO receives 16 words (one block), the HMAC core begins processing"
# Ah! In hardware, HMAC ONLY processes when it has a full 16-word (64 byte) block!
# If it has < 64 bytes, it does NOT process until HASH_PROCESS is sent!
# QEMU currently processes when it reaches 32 bytes (HMAC_FIFO_LENGTH / 2)!
# That's why it NEVER reaches 64 bytes!
# If we fix QEMU to trigger process_fifo ONLY when >= 64, it will reach 64!
# Let's check OT_HMAC_FIFO_LENGTH.
