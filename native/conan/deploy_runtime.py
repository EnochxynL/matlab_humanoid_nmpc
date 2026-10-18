"""Deploy only host DLLs, never build-tool DLLs, next to the project's MEX files."""
from pathlib import Path
import hashlib
import json
import shutil


def deploy(graph, output_folder, **kwargs):
    destination = Path(output_folder)
    destination.mkdir(parents=True, exist_ok=True)
    files = {}
    for dependency in graph.root.conanfile.dependencies.host.values():
        if not dependency.package_folder:
            continue
        # MinGW recipes may install DLLs in lib alongside their import libraries.
        for bindir in dict.fromkeys(dependency.cpp_info.bindirs + dependency.cpp_info.libdirs):
            directory = Path(dependency.package_folder) / bindir
            for source in directory.iterdir() if directory.is_dir() else []:
                if not (source.name.lower().endswith('.dll') or '.so' in source.name or source.name.endswith('.dylib')):
                    continue
                digest = hashlib.sha256(source.read_bytes()).hexdigest()
                key = source.name.lower()
                if key in files and files[key]['sha256'] != digest:
                    raise RuntimeError(f'Conflicting host DLLs: {source.name}')
                shutil.copy2(source, destination / source.name)
                files[key] = {'source': str(source), 'sha256': digest,
                              'package': str(dependency.ref)}
    (destination / 'conan-runtime.json').write_text(json.dumps(files, indent=2), encoding='utf-8')
