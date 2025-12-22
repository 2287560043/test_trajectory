#!/bin/bash

# 配置凭据帮助器，使用可自动提供凭据的方式（例如使用 cache，store，或者其他支持的方式）
git config --global credential.helper store

# 获取当前目录下的所有子模块路径
submodules=$(git submodule foreach --recursive --quiet 'echo "$PWD/$path"')

# 移除路径中的最后一个文件夹
submodules=$(echo "$submodules" | xargs -I{} dirname {} | sort | uniq)

# 去掉根目录
root_dir=$(git rev-parse --show-toplevel)
submodules=$(echo "$submodules" | sed "s|$root_dir/||")

# 循环遍历子模块并执行 git pull
for submodule in $submodules; do
    echo "Updating submodule: $submodule"
    (
        cd "$submodule"
        # 执行 git pull，注意：这里假设你的凭据已经被缓存
        git checkout master
        git pull
    )
done

git checkout master
git pull origin master 

echo "All submodules updated!"
