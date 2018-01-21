new_local_repository(
    name = "opencv",
    path = "/usr/local",
    build_file = "linux_libraries/opencv.BUILD",
)

new_http_archive(
    name = "six_archive",
    build_file = "third_party/google/protobuf/six.BUILD",
    sha256 = "105f8d68616f8248e24bf0e9372ef04d3cc10104f1980f54d57b2ce73a5ad56a",
    url = "https://pypi.python.org/packages/source/s/six/six-1.10.0.tar.gz#md5=34eed507548117b2ab523ab14b2f8b55",
)

# For protobuf. Don't use these.
bind(
    name = 'six',
    actual = '@six_archive//:six',
)
bind(
    name = 'gtest',
    actual = '//third_party/google/googletest:googlemock',
)
bind(
    name = 'gtest_main',
    actual = '//third_party/google/googletest:googlemock_main',
)

new_local_repository(
  name = "usr_python",
  path = "/usr/include/python2.7/",
  build_file = "linux_libraries/python_headers.BUILD",
)

bind(
    name = 'python_headers',
    actual = '@usr_python//:python_headers',
)
