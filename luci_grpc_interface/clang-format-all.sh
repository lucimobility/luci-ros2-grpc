#!/bin/bash

find . -regex '.*\.\(cpp\|h\)' \
    -not -path "./tclap/*" \
    -not -path "*/third_party/*" \
    -not -path "*/generated_code/*" -print0 | \
    xargs -0 -n1 -P16 clang-format-14 -style=file -i 
