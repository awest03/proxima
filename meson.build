project('proxima', 'cpp', version: '0.1.0')

proxima_src = []
subdir('src')

proxima_deps = [
    dependency('polypartition')
]

proxima_include = [
    include_directories('include')
]

proxima = library(
    'proxima',
    proxima_src,
    include_directories: proxima_include,
    dependencies: proxima_deps,
    install: true
)

proxima_dep = declare_dependency(
    link_with: proxima,
    include_directories: proxima_include,
    dependencies: proxima_deps
)

proxima_test_src = []
subdir('test')

executable(
    'proxima-test',
    proxima_test_src,
    dependencies: [
        proxima_dep
    ]
)