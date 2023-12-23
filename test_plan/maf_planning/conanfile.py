from conans import ConanFile, tools
import os
import re


class PackageConan(ConanFile):
  name = os.environ.get('PACKAGE_NAME')
  version = os.environ.get('PACKAGE_VERSION')
  settings = 'os', 'compiler', 'build_type', 'arch'
  description = 'Maf Planning'
  url = 'https://devops.momenta.works/Momenta/maf/_git/maf_planning'
  license = 'Momenta Limited, Inc'
  topics = ()
  generators = 'cmake'
  requires = (
    'mf_mlog_core/auto@momenta/stable',
    'mf_mlog_publisher/auto@momenta/stable',
    'mf_mtime_core/auto@momenta/stable',
    'mf_mtime_customize_timeline/auto@momenta/stable',
    'eigen/auto@momenta/stable',
    'maf_interface/auto@momenta/stable',
    'mf_mfruntime/auto@momenta/stable',
    'mf_mjson/auto@momenta/stable',
    'mf_mtaskflow/auto@momenta/stable',
    'ceres-solver/auto@momenta/stable',
    'boost/auto@momenta/stable',
    'nlohmann_json/auto@momenta/stable',
    'osqp/auto@momenta/stable',
    'rapidjson/auto@momenta/stable',
    'yaml-cpp/auto@momenta/stable',
    'python_alikes/auto@momenta/stable',
    'mdiag/auto@momenta/stable',
    'mf_mproto/auto@momenta/stable',
    'infer_jam/auto@momenta/stable',
    'protoc/auto@momenta/stable',
    'gflags/auto@momenta/stable',
    'websocket++/auto@momenta/stable',
    'opencv/auto@momenta/stable',
  )
  build_requires = (
    'google_test/auto@momenta/stable',
    'google_benchmark/auto@momenta/stable',
    "dbg_macro/auto@momenta/stable",
  )

  # https://docs.conan.io/en/latest/reference/conanfile/methods.html#imports
  def imports(self):
    if self.settings.os != 'Neutrino':
      self.copy('*',
                dst='deploy/common/bin',
                src='bin',
                root_package='mf_mfruntime')  # import mfrmaster
      self.copy('*.so*', dst='deploy/common/lib', src='lib')

  def requirements(self):
    if self.settings.os == 'Neutrino':
      self.requires('switch_util/auto@momenta/stable')
      self.requires('shmcom_helper/auto@momenta/stable')
    if self.project == "wulingshan":
      self.requires("maf_ap_definitions/auto@momenta/stable")

  def package(self):
    source_dir = os.environ.get('PACKAGE_SOURCE_DIR')
    build_dir = os.environ.get('PACKAGE_BUILD_DIR')
    install_dir = os.environ.get('PACKAGE_INSTALL_DIR')
    if self.project == "wulingshan":
      install_dir = os.environ.get('PACKAGE_INSTALL_DIR') + '/APAPlanningRoot'
      assert self.copy('*',
                       dst='APAPlanningRoot',
                       src=install_dir,
                       symlinks=True), 'not any APAPlanningRoot'
    else:
      assert self.copy('*.so',
                       dst='lib',
                       src='{}/lib'.format(install_dir),
                       symlinks=True), 'not any lib files'
      assert self.copy(
        '*', dst='launch',
        src='{}/launch'.format(install_dir)), 'not any launch files'
      assert self.copy(
        '*', dst='resource',
        src='{}/resource'.format(install_dir)), 'not any resource files'
      assert self.copy(
        '*', dst='scripts',
        src='{}/scripts'.format(install_dir)), 'not any scripts files' 
      self.copy(
        '*', dst='bin',
        src='{}/bin'.format(install_dir)), 'not any bin files'
    self.copy('*.so*', dst='lib', src='{}/mproto_devel/lib'.format(build_dir), symlinks=True)

  def package_info(self):
    self.cpp_info.libs = tools.collect_libs(self)

  def configure(self):

    # for platfrom:
    if str(self.settings.os) == "MDC":
      if str(self.settings.os.platform) == "610" and "mfr" not in os.getenv("BENV_ID"):
        self.options['maf_ap_applications'].plane = "A"
        self.options['maf_ap_applications'].extraversion = None
        self.options["mf_mlog_publisher"].publisher_type = "ap"
        self.options["mf_mlog_publisher"].machine_exist = False

      self.options["infer_jam"].plugins = "acl"
      self.options["infer_jam"].safe_cuda = False
      self.options["infer_jam"].cuda = None

    if str(self.settings.os) == 'Neutrino':
      if os.getenv('BUILD_SAFETY_FLAG', 'False') == 'True':
        self.requires("safety_util/auto@momenta/stable")
        self.options['infer_jam'].safe_cuda = True
        self.options["minfer"].safe_cuda = True
        self.options["safety_util"].safe_cuda = True
      else:
        self.options['infer_jam'].safe_cuda = False
        self.options["minfer"].safe_cuda = False

    try:
      if str(self.settings.os.platform) == 'Orin':
        self.options['infer_jam'].cuda = 11.4
        self.options['minfer'].cuda = 11.4
    except Exception as e:
      print("conanbuildinfo has no settings.os.platform")

    # for simulation:
    if os.environ.get('CUDA_VERSION') == "11.1":
      self.options['infer_jam'].cuda = 11.1
      self.options['minfer'].cuda = 11.1

  @property
  def project(self):
    if str(self.settings.os) == "MDC":
      if str(self.settings.os.platform) == "610":
        if "mfr" in os.getenv("BENV_ID"):
            return os.getenv("BENV_ID")
        if str(self.settings.os.version) == "1.1201.023.2":
          return "wulingshan"
        elif str(self.settings.os.version) == "1.1201.026":
          return "wulingshan"
        elif str(self.settings.os.version) == "1.1201.028":
          return "wulingshan"
        elif str(self.settings.os.version) == "1.1201.030":
          return "wulingshan"
        elif str(self.settings.os.version) == "1.1201.032.1":
          return "wulingshan"
        elif str(self.settings.os.version) == "1.1000.008":
          return "lianhuashan"
    docker_tag_id = os.getenv("BENV_ID", "others")
    if docker_tag_id == "mdc610wulingshan":
      return "wulingshan"
    elif docker_tag_id == "mdc610":
      return "lianhuashan"
    elif "mdc" in docker_tag_id:
      return "wulingshan"
    return docker_tag_id
