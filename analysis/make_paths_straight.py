from caret_analyze import Architecture

def make_paths(architecture_raw_path, architecture_target_path):
    arch = Architecture('yaml', architecture_raw_path)

    path_name_list = [
        'target_path_0',
    ]

    paths = arch.search_paths(
        '/node_src',
        '/node_dst',
    )

    for path in paths:
        path.summary.pprint()

    arch.add_path(path_name_list[0], paths[0])

    arch.export(architecture_target_path, force=True)

    return path_name_list

