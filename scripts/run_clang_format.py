import os
import subprocess

EXTENSIONS = (".c", ".cpp", ".h", ".hpp")
EXCLUDE_DIRS = {"build", ".git", "dist", "ext", "build_lib"}

for root, dirs, files in os.walk("."):
    # 除外ディレクトリをスキップ
    dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS]

    for file in files:
        if file.endswith(EXTENSIONS):
            path = os.path.join(root, file)
            print("Formatting:", path)
            subprocess.run(["clang-format", "-i", path])
