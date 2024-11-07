#!/bin/bash

# 打印大于 100 MB 的暂存文件
threshold=$((100 * 1024 * 1024))  # 100 MB in bytes

git ls-files -s | while read mode object stage file; do
    if [ -n "$object" ]; then  # 检查 object 是否非空
        size=$(git cat-file -s $object 2>/dev/null)
        if [[ "$size" =~ ^[0-9]+$ ]] && [ $size -gt $threshold ]; then
            echo "$size $file"
        fi
    fi
done
