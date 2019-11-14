import os
import re
import subprocess
import time

current_dir = os.path.abspath(os.path.dirname(__file__)) + "/"

file_ends = (r'.cpp', r'.h', r'.hpp', r'.hxx', r'.c', r'cxx')
file_dict = {}
file_list = ""
with open(current_dir + "FileList", 'r') as flist:
    for f in flist.readlines():
        if (f.endswith('\n')):
            f = f[:-1]
        if re.findall(r"[.cpp|.hpp|.c|.h|.cxx|.hxx]$", f):
            file_dict[f] = ""
            print(f)

# Traverse workspace to find related files in FileList
for dirpath, dirnames, filenames in os.walk(current_dir + "/../"):
    for filename in filenames:
        input_file = os.path.basename(filename)
        if file_dict.__contains__(input_file):
            file_dict[input_file] = os.path.join(dirpath, filename)

# Doxygen filelist
for k, v in file_dict.items():
    file_list += v + " "

# Modify doxygen config
print("Phase 1: " + "Modify configuration")
prefix = "INPUT   = "
cmd = "gsed -i \'774c " + prefix + file_list + "\' " + current_dir + "Doxyfile"
config_result = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
while subprocess.Popen.poll(config_result) is None:
    time.sleep(0.1)

# Generate docs to html and latex
print("Phase 2: " + "Generate docs")
gen_doxygen_cmd = "doxygen " + current_dir + "Doxyfile"
gen_result = subprocess.Popen(gen_doxygen_cmd, stdout=subprocess.PIPE, shell=True)
while subprocess.Popen.poll(gen_result) is None:
    time.sleep(0.1)

# Generate PDF in Chinese
print("Phase 3: " + "Set utf-8 for Chinese pdf")
replace_cmd = r"gsed -i -e 's,begin{document},usepackage{CJKutf8}\n\\begin{document}\n\\begin{CJK}{UTF8}{gbsn},' " + current_dir + r"/latex/refman.tex"
print(replace_cmd)
replace_result = subprocess.Popen(replace_cmd, stdout=subprocess.PIPE, shell=True)
while subprocess.Popen.poll(replace_result) is None:
    time.sleep(0.1)


replace_cmd = r"gsed -i -e 's,end{document},end{CJK}\n\\end{document},' " + current_dir + r"/latex/refman.tex"
print(replace_cmd)
replace_result = subprocess.Popen(replace_cmd, stdout=subprocess.PIPE, shell=True)
while subprocess.Popen.poll(replace_result) is None:
    time.sleep(0.1)

# Sleep until the replace operation finish
print("Phase 4: " + "Transform latex to pdf")
make_cmd = r"cd " + current_dir + r"/latex/;" + r"make -k"
print(make_cmd)
make_result = subprocess.Popen(make_cmd, stdout=subprocess.PIPE, shell=True)
out, err = make_result.communicate()
while subprocess.Popen.poll(make_result) is None:
    print("Not finished")
    time.sleep(0.1)

