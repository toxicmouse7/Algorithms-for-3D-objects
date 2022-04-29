import os
from subprocess import Popen, DEVNULL


def remove_ds_store(collection: list):
    while '.DS_Store' in collection:
        collection.remove('.DS_Store')


def main():
    directories = os.listdir()
    directories.remove('cows')
    directories.remove('main.py')
    remove_ds_store(directories)
    directory_count = 0

    for ply_directory in directories:
        ply_clouds = os.listdir(ply_directory)
        remove_ds_store(ply_clouds)
        processes = []
        for ply_cloud in ply_clouds:
            if not ply_cloud.endswith('.ply'):
                continue
            if not os.path.exists(f'../right_projection/{ply_directory}'):
                os.mkdir(f'../right_projection/{ply_directory}')
            processes.append(
                Popen([
                    './cows',
                    f'{ply_directory}/{ply_cloud}',
                    '4',
                    f"../right_projection/{ply_directory}/{ply_cloud.replace('.ply', '.bmp')}"
                ],
                      stdout=DEVNULL, stderr=DEVNULL)
            )
        for process in processes:
            process.wait()
        directory_count += 1
        print(f'[{directory_count}/{len(directories)}] {ply_directory} Done!')

    print('Task finished')


if __name__ == '__main__':
    main()
