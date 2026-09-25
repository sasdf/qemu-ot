#!/bin/bash
set -e
cd /root/src/workspaces/eb076343-f08d-41c9-baf6-c97f682b4cb6/opentitan
bazelisk test //sw/device/tests/qemu/errata:aon_timer_errata_test_sim_qemu_rom_with_fake_keys --nocache_test_results
