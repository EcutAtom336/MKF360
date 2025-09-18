import os
import wave
import struct


def check_wav_format(wav_path):
    try:
        with wave.open(wav_path, "rb") as wav:
            return (
                wav.getnchannels() == 1
                and wav.getsampwidth() == 2
                and wav.getframerate() == 16000
            )
    except:
        return False


def process_wav_files():
    # 配置参数
    wav_dir = "./audio"
    pcm_dir = "./audio/src"
    include_dir = "./audio/include/audio"
    header_file = os.path.join(include_dir, "PCM_RES.h")

    # 创建目录
    os.makedirs(pcm_dir, exist_ok=True)
    os.makedirs(include_dir, exist_ok=True)

    # 获取WAV文件列表
    try:
        wav_files = [
            f
            for f in os.listdir(wav_dir)
            if f.lower().endswith(".wav") and os.path.isfile(os.path.join(wav_dir, f))
        ]
    except FileNotFoundError:
        print(f"Error: Directory {wav_dir} does not exist")
        return

    # 按文件名排序
    wav_files.sort()

    # 处理每个WAV文件
    pcm_data_info = []
    for idx, wav_file in enumerate(wav_files):
        wav_path = os.path.join(wav_dir, wav_file)

        if not check_wav_format(wav_path):
            print(f"Skipping {wav_file}: Not 16bit 16kHz mono format")
            continue

        # 读取PCM数据
        with wave.open(wav_path, "rb") as wav:
            frames = wav.readframes(wav.getnframes())
            pcm_data = list(struct.unpack(f"<{len(frames)//2}h", frames))

        # 生成C文件名
        base_name = os.path.splitext(wav_file)[0].upper()
        var_name = f"{base_name}_PCM"
        c_file_name = f"{var_name}.c"
        c_file_path = os.path.join(pcm_dir, c_file_name)

        # 写入C文件
        if os.path.exists(c_file_path):
            print(f"Overwriting {c_file_path}")

        with open(c_file_path, "w", encoding="utf-8") as f:
            f.write("#include <stdint.h>\n\n")
            f.write(f"const int16_t {var_name}[{len(pcm_data)}] = {{\n")

            # 每行写入10个数据
            for i in range(0, len(pcm_data), 10):
                line = pcm_data[i : i + 10]
                f.write("    " + ", ".join(str(x) for x in line) + ",\n")

            f.write("};\n")

        # 保存信息用于头文件
        pcm_data_info.append(
            {"var_name": var_name, "length": len(pcm_data), "index": idx}
        )

    # 生成头文件
    if os.path.exists(header_file):
        print(f"Overwriting {header_file}")

    with open(header_file, "w", encoding="utf-8") as f:
        f.write("#ifndef __PCM_RES_H__\n")
        f.write("#define __PCM_RES_H__\n\n")
        f.write("#include <stdint.h>\n\n")

        # 写入长度宏定义和索引宏定义
        for info in pcm_data_info:
            var_name = info["var_name"]
            f.write(f'#define {var_name}_LEN ((uint32_t){info["length"]}U)\n')
            f.write(f'#define {var_name}_IDX ((uint8_t){info["index"]}U)\n')

        # 写入NONE_PCM_IDX
        f.write(f"\n#define NONE_PCM_IDX ((uint8_t){len(pcm_data_info)}U)\n\n")

        # 写入外部引用
        for info in pcm_data_info:
            f.write(
                f'extern const int16_t {info["var_name"]}[{info["var_name"]}_LEN];\n'
            )

        f.write("\n#endif // __PCM_RES_H__\n")

    print(f"Processed {len(pcm_data_info)} WAV files")


if __name__ == "__main__":
    process_wav_files()
