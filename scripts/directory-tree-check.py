#! /bin/python3

from functools import reduce
import os
import pprint

ignore_set = ('CMakeLists.txt', 'README.md', 'package.xml')


def check_pkg_readme(pkg_dir, dir_tree):
    pp = pprint.PrettyPrinter(indent=2)
    print(pkg_dir)
    pp.pprint(dir_tree)

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
