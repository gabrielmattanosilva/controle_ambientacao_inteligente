import os

def concatenate_c_h_files_in_lib_src(base_dir="."):
    target_dirs = ["include", "lib", "src"]
    concatenated_files = []

    # Adicionar o platformio.ini se existir
    ini_path = os.path.join(base_dir, "platformio.ini")
    if os.path.isfile(ini_path):
        with open(ini_path, "r", encoding="utf-8") as f:
            content = f.read()
            header = f"\n/********** {os.path.relpath(ini_path, base_dir)} **********/\n\n"
            concatenated_files.append(header + content)

    # Percorrer as pastas alvo
    for folder in target_dirs:
        full_path = os.path.join(base_dir, folder)
        if not os.path.isdir(full_path):
            continue  # pular se a pasta não existir

        for root, dirs, files in os.walk(full_path):
            for filename in sorted(files):
                if filename.endswith((".c", ".cpp", ".h")):
                    file_path = os.path.join(root, filename)
                    with open(file_path, "r", encoding="utf-8") as f:
                        content = f.read()
                        header = f"\n/********** {os.path.relpath(file_path, base_dir)} **********/\n\n"
                        concatenated_files.append(header + content)

    # Nome do arquivo de saída
    base_name = os.path.basename(os.path.abspath(base_dir.rstrip("/\\")))
    output_file = f"{base_name}.txt"

    with open(output_file, "w", encoding="utf-8") as out_file:
        out_file.write("\n".join(concatenated_files))

    print(f"Concatenated files saved to: {output_file}")

def main():
    base_directory = "."  # altere se necessário
    concatenate_c_h_files_in_lib_src(base_directory)

if __name__ == "__main__":
    main()