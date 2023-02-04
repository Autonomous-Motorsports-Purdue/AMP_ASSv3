#! /bin/python3

from functools import reduce
import re
import sys
import os

ignore_set = ('CMakeLists.txt', 'README.md', 'package.xml')

start_marker = '<!-- directory-tree-check-start -->\n'
end_marker = '<!-- directory-tree-check-end -->\n'


def check_pkg_readme(pkg_dir, dir_tree):
    with open(os.path.join(pkg_dir, 'README.md'), 'r') as f:
        lines = f.readlines()
        md_tree = ''.join(lines[lines.index(start_marker) +
                                2:lines.index(end_marker) - 1])

    if dir_tree != get_dir_tree_from_md(md_tree):
        sys.exit(f'Package directory "{pkg_dir}" README is not updated.')


def get_dir_tree_from_md(md_tree):
    print(md_tree)
    pass


def main():
    dir_trees = {}

    os.chdir('./src')
    for path, _, files in os.walk('.'):
        # ignore non-AMP directories
        if not path.startswith('./amp_'):
            continue

        folders = path[2:].split(os.sep)
        sub_dirs = dict.fromkeys([f for f in files if f not in ignore_set])
        parent = reduce(dict.get, folders[:-1], dir_trees)
        parent[folders[-1]] = sub_dirs

    for pkg_dir, dir_tree in dir_trees.items():
        check_pkg_readme(pkg_dir, dir_tree)


if __name__ == "__main__":
    main()
