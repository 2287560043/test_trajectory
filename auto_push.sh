#!/bin/bash

# 配置凭据帮助器，以缓存凭据（输入一次密码后，之后的推送不再需要密码）
git config --global credential.helper store

# 获取当前目录下的所有子模块路径，并移除主仓库名字
submodules=$(git submodule foreach --recursive --quiet 'echo "$PWD/$path"')

# 移除路径中的最后一个文件夹
submodules=$(echo "$submodules" | xargs -I{} dirname {} | sort | uniq)

# 去掉根目录
root_dir=$(git rev-parse --show-toplevel)
submodules=$(echo "$submodules" | sed "s|$root_dir/||")

# 循环遍历子模块并检查是否有待提交的更改
for submodule in $(echo "$submodules" | tac); do
    (
        # 进入目录
        cd "$root_dir/$submodule" || exit
        # 检查是否有未提交的更改
        if [[ -n $(git status --porcelain) ]]; then
            # 接受用户输入的 commit message
            read -p "Enter commit message for $submodule: " commit_message
            # 执行 git add 和 git commit
            git add .
            git commit -m "$commit_message"
            # 执行 git push
            git push
        else
            echo "No changes in $submodule"
        fi
    )
done

# 如果有顶层仓库的更改需要提交，则进行提交和推送
if [[ -n $(git status --porcelain) ]]; then
    echo "Pushing changes in top submodule"
    # 接受用户输入的 commit message
    read -p "Enter commit message for top submodule: " commit_message
    git add .
    git commit -m "$commit_message"
    git push origin master
else
    echo "No changes in top submodule"
fi

echo "All submodules pushed"
