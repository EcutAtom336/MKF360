#!/usr/bin/env python3
"""
clang-format 批量格式化工具 (多目录硬编码版)
用法: cf_format.py --clang-format-exe <路径>
"""

import os
import subprocess
import argparse
from pathlib import Path

# 相对于脚本上一层目录的目标目录
TARGET_DIRS = [
    Path("STM32CubeMX/Target_1/STM32CubeMX/Src"),
    Path("STM32CubeMX/Target_1/STM32CubeMX/Inc"),
    # 可以添加更多目录...
]
CONFIG_NAME = ".clang-format"  # 配置文件名


def format_directory(clang_format_exe: str, verbose: bool = True):
    """
    格式化多个硬编码目录下的C/C++文件
    """
    # 获取脚本所在目录的上一层目录
    script_dir = Path(__file__).resolve().parent
    work_dir = script_dir.parent
    os.chdir(work_dir)  # 强制切换工作目录

    config_file = work_dir / CONFIG_NAME

    # 验证配置文件存在性
    if not config_file.exists():
        raise FileNotFoundError(f"配置文件不存在: {config_file}")

    # 检查clang-format可用性
    try:
        subprocess.run(
            [clang_format_exe, "--version"],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception as e:
        raise RuntimeError(f"clang-format不可用({clang_format_exe}): {str(e)}")

    total_files = 0
    total_success = 0

    for target_dir in TARGET_DIRS:
        target_path = work_dir / target_dir
        if not target_path.exists():
            print(f"⚠️ 目录不存在，已跳过: {target_path}")
            continue

        files = list(target_path.glob("*.c")) + list(target_path.glob("*.h"))
        if not files:
            if verbose:
                print(f"ℹ️ 目录中未找到.c/.h文件: {target_path}")
            continue

        if verbose:
            print(f"\n🔍 在目录 {target_path} 中找到 {len(files)} 个文件，开始格式化...")

        success = 0
        for file in files:
            try:
                if verbose:
                    print(f"🛠️  正在处理: {file.relative_to(target_path)}")

                subprocess.run(
                    [clang_format_exe, "-i", f"--style=file:{config_file}", str(file)],
                    check=True,
                )
                success += 1
            except subprocess.CalledProcessError as e:
                print(f"❌ 格式化失败: {file.relative_to(target_path)} ({str(e)})")
            except Exception as e:
                print(f"⛔ 处理出错: {file.relative_to(target_path)} ({str(e)})")

        total_files += len(files)
        total_success += success

        if verbose:
            print(f"✅ 目录 {target_path} 完成! 成功处理 {success}/{len(files)} 个文件")

    if verbose:
        print(f"\n🎉 所有目录处理完成! 总计成功 {total_success}/{total_files} 个文件")


def main():
    parser = argparse.ArgumentParser(
        description="批量格式化C/C++文件 (多目录硬编码版)",
        formatter_class=argparse.RawTextHelpFormatter,
    )

    parser.add_argument(
        "--clang-format-exe",
        required=True,
        help="clang-format可执行文件路径 (必须指定)",
    )
    parser.add_argument(
        "-q", "--quiet", action="store_true", help="安静模式 (减少输出)"
    )

    args = parser.parse_args()

    try:
        format_directory(clang_format_exe=args.clang_format_exe, verbose=not args.quiet)
    except Exception as e:
        print(f"💥 错误: {str(e)}")
        exit(1)


if __name__ == "__main__":
    main()
