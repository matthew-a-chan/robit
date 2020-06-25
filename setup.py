#! /usr/local/bin/python3

import subprocess

paths = ['simulation/', '*.py']
joined_paths = ' '.join(paths)


def main():

    pre_commit = f'''#!/bin/sh


    yapf --recursive --quiet {joined_paths}
    RET_CODE=$?

    yapf --recursive --in-place {joined_paths}

    if [ ! $RET_CODE == 0 ]
    then
        echo Formatting was bad!! Fixing now.
        git reset
        exit $RET_CODE
    else
        echo Formatting all good! :D
        exit 0
    fi
    '''
    with open('.git/hooks/pre-commit', 'w') as pre_commit_file:
        pre_commit_file.write(pre_commit)

    subprocess.run(['chmod', '744', '.git/hooks/pre-commit'])


if __name__ == '__main__':
    main()
