import os
import re
import subprocess


current_dir = os.path.abspath(os.path.dirname(__file__)) + "/"

file_ends = (r'.cpp', r'.h', r'.hpp', r'.hxx', r'.c', r'cxx')
file_dict = {}
file_list = ""
with open(current_dir + "FileList", 'r') as flist:
    for f in flist.readlines():
        if(f.endswith('\n')):
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
prefix = "INPUT   = "
cmd = "sed -i \'774c " + prefix + file_list + "\' " + current_dir + "Doxyfile"
subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)

# Generate docs to html
gen_doxygen_cmd = "doxygen " + current_dir + "Doxyfile"
gen_result = subprocess.Popen(gen_doxygen_cmd, stdout=subprocess.PIPE, shell=True)
out, err = gen_result.communicate()
print(out)
