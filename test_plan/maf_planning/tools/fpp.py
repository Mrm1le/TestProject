#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import logging
import sys
import getpass
import subprocess
import json
import time
import re
import pathlib
import docker
from datetime import datetime
import os
import traceback
import multiprocessing
import socket
import urllib

default_output_dir = './tmp/{0}/npp/{1}'.format(getpass.getuser(), datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))

parser = argparse.ArgumentParser(description='This tool is called "fastest planning player" (named by c++ master). indro: https://momenta.feishu.cn/wiki/wikcnUt0ddk3J3HRczNj3Iun4jf')

parser.add_argument('--planning-offboard-url', dest='offboard_url', action='store', type=str, required=False, default='https://devops.momenta.works/Momenta/msd-planning/_git/planning_offboard', help='offboard url')
parser.add_argument('--planning-offboard-branch', dest='offboard_branch', action='store', type=str, required=False, default='dev', help='offboard branch')
parser.add_argument('--planning-offboard-depth', dest='offboard_depth', action='store', type=int, required=False, default=1, help='0 - non-shadow; >=1 - shadow')
parser.add_argument('--build', dest='build', action='store_true', default=False, help='build, deploy if not deployed before')
parser.add_argument('--deploy', dest='deploy', action='store_true', default=False, help='build, deploy')
parser.add_argument('--play', dest='bag', action='store', type=str, required=False, default='', help='play bag, pass bag here')
parser.add_argument('--mviz', dest='mviz', action='store_true', required=False, default=False, help='generate mviz')
parser.add_argument('--debug', dest='debug', action='store_true', required=False, default=False, help='play bag in debug mode')
parser.add_argument('--trace-throw', dest='trace_throw', action='store_true', required=False, default=False, help='trace throw')
parser.add_argument('--frame', dest='frame', required=False, type=int, default=-1, help='break at which frame, only effect in debug mode')
parser.add_argument('--stress', dest='stress', action='store_true', required=False, default=False, help='play bag in stress test mode')
parser.add_argument('--perf', dest='perf', action='store_true', required=False, default=False, help='use perf generate flame graph')
parser.add_argument('--start-container', dest='start_container', action='store_true', required=False, default=False, help='restart fpp conatiner')
parser.add_argument('--stop-container', dest='stop_container', action='store_true', required=False, default=False, help='stop fpp conatiner')
parser.add_argument('--attach', dest='attach', action='store_true', required=False, default=False, help='attach fpp conatiner')
parser.add_argument('--attach-ros', dest='attach_ros', action='store_true', required=False, default=False, help='attach ros conatiner')
parser.add_argument('--reset-env', dest='reset_env', action='store_true', required=False, default=False, help='rm build dir and docker container')
parser.add_argument('--maf-planning-dir', dest='maf_planning_dir', required=False, default='', help='path to maf_planning dir')
parser.add_argument('--fpp-image', dest='fpp_image', required=False, default='', help='runtime image')
parser.add_argument('--gpu', dest='gpu', action='store_true', required=False, default=False, help='use gpu')
parser.add_argument('--closed-loop', dest='closed_loop', action='store_true', required=False, default=False, help='run closed loop')
parser.add_argument('--compare-mviz', dest='compare_mviz', action='store_true', required=False, default=False, help='generate convert link')
parser.add_argument('--mfr-port', dest='mfr_port', action='store', required=False, default='11300', help='mfr master port')
parser.add_argument('--ros-port', dest='ros_port', action='store', required=False, default='11301', help='ros master port')
parser.add_argument('--pnp', dest='pnp', action='store_true', required=False, default=False, help='send pnp scenario')
parser.add_argument('--apa', dest='apa', action='store_true', required=False, default=False, help='apa scenario')
parser.add_argument('--auto-time', dest='auto_time', required=False, default=None, help="closed loop start auto time")
args = parser.parse_args()

logging.basicConfig(stream=sys.stdout,
        level=logging.INFO,
        format='%(asctime)s  %(name)s  %(levelname)s: %(message)s')

if args.maf_planning_dir == '':
    args.maf_planning_dir = str(pathlib.Path(__file__).parent.parent.absolute())
else:
    args.maf_planning_dir = str(pathlib.Path(args.maf_planning_dir).resolve())
logging.info('maf_planning_dir: %s', args.maf_planning_dir)

mviz_creator_url = 'https://artifactory.momenta.works/artifactory/msim-tools/release/latest/sim'
mviz_creator_path = f'{args.maf_planning_dir}/tools/bin/sim'
pathlib.Path(mviz_creator_path).parent.mkdir(parents=True, exist_ok=True)

if args.bag != '':
    args.bag = str(pathlib.Path(args.bag).resolve())
logging.info('bag: %s', args.bag)

if args.fpp_image == '':
    args.fpp_image = 'artifactory.momenta.works/docker-momenta/np_planning/fpp_local:maf3.2.1-5'
logging.info('fpp_image: %s', args.fpp_image)

fpp_bin_path='/opt/maf_planning/build/tools/calc_one_frame/fpp_sim'
fpp_runtime_env=''
if args.gpu:
    fpp_runtime_env='LD_LIBRARY_PATH=${HOME}/.iso_compiler/v2/cuda-x86_64-11.1/usr/local/cuda-11.1/targets/x86_64-linux/lib/:$LD_LIBRARY_PATH '

if args.trace_throw:
    fpp_runtime_env = f'LD_PRELOAD=/opt/maf_planning/build/tools/trace_throw/libtrace_throw.so {fpp_runtime_env}'

fpp_bin_args=''
if args.auto_time:
    fpp_bin_args += f' --shadow-mode={args.auto_time}'
if args.closed_loop:
    fpp_bin_args += " --is-closed-loop=1"
if args.pnp:
    fpp_bin_args += " --is_pnp=1"
if args.apa:
    fpp_bin_args += " --is_apa=1"

logging.info('run on gpu: %d', args.gpu)

logging.info('build: %s', args.build)
logging.info('deploy: %s', args.deploy)
logging.info('frame: %s', args.frame)

fpp_container_name='fpp-container-{0}'.format(args.maf_planning_dir.replace('/', '-').strip('-'))
logging.info('fpp_container_name: %s', fpp_container_name)

ros_container_name='ros-adapter-{0}'.format(fpp_container_name)

def start_container(image, name=None, options=None, network=None, envs=None, volumes=None, cmd=None):
    uid = subprocess.check_output(['id', '-u']).decode().strip()
    gid = subprocess.check_output(['id', '-g']).decode().strip()
    run_cmd = f'docker run --user {uid}:{gid}'

    if name:
        run_cmd += ' --name {0}'.format(name)
    if options:
        run_cmd += ' ' + ' '.join(options)
    if network:
        run_cmd += ' --network {0}'.format(network)
    if envs:
        for env in envs:
            if env[1]:
                run_cmd += ' -e {0}={1}'.format(env[0], env[1])
            else:
                run_cmd += ' -e {0}'.format(env[0])
    if volumes:
        for volume in volumes:
            if not pathlib.Path(volume[0]).is_file():
                pathlib.Path(volume[0]).mkdir(parents=True, exist_ok=True)
            run_cmd += ' -v {0}:{1}'.format(volume[0], volume[1])
    run_cmd += ' ' + image
    if cmd:
        run_cmd += ' ' + cmd

    logging.info("create container: {0}".format(run_cmd))
    os.system(run_cmd)

def attach_container(name=None, cmd=None):
    run_cmd = 'docker exec -it'
    run_cmd += ' ' + name
    if cmd:
        run_cmd += ' ' + cmd

    logging.info("attaching container: {0}".format(run_cmd))
    subprocess.run(run_cmd.split(' '), capture_output=False, encoding='UTF-8')

def start_fpp_container():
    logging.info('start container: %s', fpp_container_name)
    proc = subprocess.Popen(f'make -C {args.maf_planning_dir} start_fpp', stdin=subprocess.PIPE, shell=True)
    proc.communicate(input='exit\n'.encode())
    ret = proc.wait(timeout=5)
    if ret != 0:
        raise Exception(f'Failed to start_fpp_container')

def start_ros_container():
    logging.info('start ros container: %s', ros_container_name)

    options=['-tid', '--rm', '-w /root/script']
    ros_image = 'artifactory.momenta.works/docker-mpilot-highway-dev/ros_mfr_adaptor:maf3.1_mfr2.5.13-3a7febe-maf3.1_0608-3e13f34'

    start_container(ros_image,
        name=ros_container_name,
        options=options,
        envs=[
            ('MY_MFR_IP', '127.0.0.1'),
            ('MY_MFR_PORT', args.mfr_port),
            ('MY_ROS_IP', '127.0.0.1'),
            ('MY_ROS_PORT', args.ros_port)
            ],
        volumes=[
            (args.maf_planning_dir, '/opt/maf_planning'),
            ('{}/adapter/mfr/script'.format(args.maf_planning_dir), '/root/script'),
            ('{}/adapter/mfr/launch/resource/msim_adaptor.json'.format(args.maf_planning_dir), '/home/ros/catkin_ws/install/share/ros_mfr_adaptor/resource/msim_adaptor.json'),
            ('{}/bags'.format(args.maf_planning_dir), '/root/Downloads'),
            ('/dev/shm', '/dev/shm'),
            ('/etc/localtime', '/etc/localtime'),
            ('{}/.gitconfig'.format(os.environ['HOME']), '/root/.gitconfig'),
            ('{}/.git-credentials'.format(os.environ['HOME']), '/root/.git-credentials')
            ],
        network='host',
        cmd='bash')

def attach_ros_container():
    if not is_container_running(ros_container_name):
        if is_container_stopped(ros_container_name):
            remove_container(ros_container_name)
        start_ros_container()
    attach_container(name=ros_container_name, cmd='bash')
    logging.info('ros container exited')

def attach_fpp_container():
    logging.info('attach container: %s', fpp_container_name)
    check_to_start_fpp_container()
    attach_container(name=fpp_container_name, cmd='bash')
    logging.info('fpp container exited')

def is_container_running(name):
    docker_client = docker.from_env()
    try:
        container = docker_client.containers.get(name)
    except docker.errors.NotFound:
        return False
    else:
        container_state = container.attrs["State"]
        return container_state["Status"] == 'running'

def is_same_image(image_name, container_name):
    if not is_container_running(container_name):
        raise Exception(f'container {container_name} is not running')

    docker_client = docker.from_env()
    container = docker_client.containers.get(container_name)
    return image_name in container.image.tags

def is_container_stopped(name):
    docker_client = docker.from_env()
    try:
        container = docker_client.containers.get(name)
    except docker.errors.NotFound:
        return False
    else:
        container_state = container.attrs["State"]
        return container_state["Status"] != 'running'

def stop_container(name):
    try:
        docker_client = docker.from_env()
        container = docker_client.containers.get(name)
        logging.info('stop container: %s', fpp_container_name)
        container.stop()
        time.sleep(3)
    except docker.errors.NotFound:
        logging.info('container %s not running', fpp_container_name)

def restart_container(name):
    docker_client = docker.from_env()
    container = docker_client.containers.get(name)
    logging.info('restart container: %s', fpp_container_name)
    container.restart()

def remove_container(name):
    try:
        docker_client = docker.from_env()
        container = docker_client.containers.get(name)
        logging.info('remove container: %s', name)
        container.remove()
    except docker.errors.NotFound:
        logging.info('container %s not running', name)

def deploy_fpp():
    logging.info('deploy fpp')
    cmd = [
        'docker', 'exec', '-ti', fpp_container_name, 'bash', '-c', 'cd /opt/maf_planning; make local_fpp_deploy'
    ]

    logging.info(' '.join(cmd))
    subprocess.call(cmd)

def build_fpp():
    logging.info('start build fpp')
    cmd = [
        'docker', 'exec', '-ti', fpp_container_name, 'bash', '-c', 'cd /opt/maf_planning; make ci_build'
    ]

    logging.info(' '.join(cmd))
    subprocess.call(cmd)
    logging.info('end build fpp')

def build_deploy():
    build_fpp()

    planning_package_path = f'{args.maf_planning_dir}/build/deploy.tar.gz'
    if args.deploy or (not pathlib.Path(planning_package_path).is_file()):
        deploy_fpp()

def check_to_start_fpp_container():
    if is_container_running(fpp_container_name):
        if args.build or args.deploy:
            build_deploy()

        logging.info('use exist container: %s', fpp_container_name)
        return

    if is_container_stopped(fpp_container_name):
        remove_container(fpp_container_name)

    start_fpp_container()
    if args.build or args.deploy:
        build_deploy()

def check_to_prepare_bag_file_in_host():
    if not pathlib.Path(args.bag).is_file():
        raise Exception('failed to find bag file: %s', args.bag)
    tmp_bag_file_path_in_host = str((pathlib.Path(f'{args.maf_planning_dir}/bags/tmp') / pathlib.Path(args.bag).name).resolve())
    bag_file_path_in_host = str(pathlib.Path(args.bag).resolve())
    if tmp_bag_file_path_in_host == bag_file_path_in_host:
        return

    pathlib.Path(tmp_bag_file_path_in_host).parent.mkdir(parents=True, exist_ok=True)
    if pathlib.Path(tmp_bag_file_path_in_host).is_file():
        logging.info('use exist bag: %s', tmp_bag_file_path_in_host)
        return
    ret = subprocess.call((f'ln {bag_file_path_in_host} {tmp_bag_file_path_in_host}').split())
    if ret != 0:
        subprocess.check_call((f'cp {bag_file_path_in_host} {tmp_bag_file_path_in_host}').split())

def check_to_prepare_bag():
    if args.bag == '':
        return get_last_bag_file_path_in_container()

    check_to_prepare_bag_file_in_host()

    bag_file_path_in_container = str(pathlib.Path('/opt/maf_planning/bags/tmp') / pathlib.Path(args.bag).name)

    return bag_file_path_in_container

def get_bag_file_path_in_container():
    bag_file_path_in_container = check_to_prepare_bag()
    if bag_file_path_in_container == '':
        raise Exception('must set bag to play with')
    return bag_file_path_in_container

def play_bag(bag_file_path_in_container):
    planning_log_path_in_host = f'{args.maf_planning_dir}/logs/planning.log'
    pathlib.Path(planning_log_path_in_host).parent.mkdir(parents=True, exist_ok=True)
    planning_log_path_in_container = '/opt/maf_planning/logs/planning.log'
    cmd = [
        'docker', 'exec', '-ti', fpp_container_name, 'bash', '-c', f'{fpp_runtime_env} {fpp_bin_path} -calc-one-frame {bag_file_path_in_container} {fpp_bin_args} >{planning_log_path_in_container} 2>&1' ]
    logging.info('begin play bag: %s', ' '.join(cmd))

    try:
        start_time = time.time()
        if args.gpu:
            # TODO: inferjam exception not tobe fixed
            subprocess.run(cmd, check=False, capture_output=True, encoding='UTF-8')
        else:
            subprocess.run(cmd, check=False, capture_output=True, encoding='UTF-8')
        endtime_time = time.time()
        logging.info('end play bag, elapsed: {:.3f}s'.format(endtime_time - start_time))
        logging.info('planning log path: %s', planning_log_path_in_host)

        tmp_bag_file_path_in_host = str((pathlib.Path(f'{args.maf_planning_dir}/bags/tmp') / pathlib.Path(args.bag).name).resolve())
        plan_file_path = f'{tmp_bag_file_path_in_host}.plan'
        logging.info('plan file path: %s', plan_file_path)
    except Exception:
        print(traceback.format_exc())

        print('\nfpp log:')
        os.system(f'docker exec -ti {fpp_container_name} tail -n50 /opt/planning.log')
        exit(1)

    return plan_file_path

def stress_test(bag_file_path_in_container):
    cmd = [
        'docker', 'exec', '-ti', fpp_container_name, 'bash', '-c', f'NP_LOG_LEVEL=5 {fpp_runtime_env} {fpp_bin_path} -stress_test {bag_file_path_in_container}'
    ]
    logging.info('begin stress test: %s', ' '.join(cmd))

    with subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=False) as proc:
        while True:
            line = proc.stdout.readline().decode('utf-8')
            if not line:
                sys.stdout.write('\n')
                break
            if line[0] != '[': # skip mlog
                sys.stdout.write(line)

    logging.info('end stress test')

def check_to_fetch_mviz_creator():
    if pathlib.Path(mviz_creator_path).is_file():
        os.system(f'cd {str(pathlib.Path(mviz_creator_path).parent)}; ./{pathlib.Path(mviz_creator_path).name} update')
        os.system(f'chmod +x {mviz_creator_path}')
        return

    logging.info('download file from %s to %s', mviz_creator_url, mviz_creator_path)
    os.system(f'curl -L {mviz_creator_url} -o {mviz_creator_path}')
    os.system(f'chmod +x {mviz_creator_path}')

    if not pathlib.Path(mviz_creator_path).is_file():
        logging.error('failed to download file from %s', mviz_creator_url)
        sys.exit(1)

def gen_origin_mviz():
    logging.info('generating origin mviz')
    check_to_fetch_mviz_creator()
    mfbag_flag = ''
    if pathlib.Path(args.bag).suffix == '.mfbag':
        mfbag_flag = '--mfbag'
    cmd = [
        mviz_creator_path,
        'viz',
        '-i',
        args.bag
    ]

    mviz_link = ""
    with subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=False) as proc:
        while True:
            line = proc.stdout.readline().decode('utf-8')
            if args.compare_mviz:
                if "https://mviz.momenta.works/player" in line:
                    mviz_link = line
            if not line:
                sys.stdout.write('\n')
                break
            sys.stdout.write(line)
    return mviz_link

def get_url_query_param(url, key):
    url_obj = urllib.parse.urlparse(url)
    return urllib.parse.parse_qs(url_obj.query)[key][0]

def gen_compare_mviz_link(lhs_url, rhs_url):
    url_obj = urllib.parse.urlparse(lhs_url)
    query_params = dict()
    query_params['bags'] = ['{0},{1}'.format(get_url_query_param(lhs_url, 'path'), get_url_query_param(rhs_url, 'path'))]
    query_params['format'] = ['multi_pbe_gz']
    query_params['user'] = ['MSD']
    query_params['zip'] = ['20']
    query_params['st'] = ['0,0']
    query_str = urllib.parse.urlencode(query_params, doseq=True)
    new_url_obj = url_obj._replace(query=query_str,path='/multiplayer/v4/')
    return new_url_obj.geturl()

def gen_mviz(plan_file_path):
    logging.info('begin gen mviz')
    check_to_fetch_mviz_creator()
    mfbag_flag = ''
    if pathlib.Path(plan_file_path).suffix == '.mfbag':
        mfbag_flag = '--mfbag'
    cmd = [
        mviz_creator_path,
        'viz',
        '-i',
        plan_file_path
    ]

    with subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=False) as proc:
        while True:
            line = proc.stdout.readline().decode('utf-8')
            if args.compare_mviz:
                if "https://mviz.momenta.works/player" in line:
                    new_bag_mviz = line
            if not line:
                sys.stdout.write('\n')
                break
            sys.stdout.write(line)

    logging.info('end gen mviz')
    if args.compare_mviz:
        origin_bag_mviz = gen_origin_mviz()
        if origin_bag_mviz :
            compare_mviz_link = gen_compare_mviz_link(origin_bag_mviz, new_bag_mviz)
            logging.info(f'mviz compare link: {compare_mviz_link}')

def debug(bag_file_path_in_container):
    cmd = f'docker exec -ti {fpp_container_name} bash -c "{fpp_runtime_env} gdb --args {fpp_bin_path} -calc-one-frame {bag_file_path_in_container} -debug_frame {args.frame} {fpp_bin_args}"'
    logging.info('debug cmd: %s', cmd)
    os.system(cmd)

def rm_vscode_container_dir():
    logging.info('begin rm_vscode_container_dir')
    cmd = [
        'docker',
        'exec',
        '-i',
        fpp_container_name,
        'bash',
        '-c',
        'cd /opt/maf_planning; rm -rf .vscode-container'
    ]
    subprocess.run(cmd)
    logging.info('end rm_vscode_container_dir')

def rm_fpp_container():
    logging.info('begin rm_fpp_container: %s', fpp_container_name)
    if is_container_running(fpp_container_name):
        restart_container(fpp_container_name) # kill processes in container
        rm_vscode_container_dir()
        stop_container(fpp_container_name)

    logging.info('end rm_fpp_container')

def reset_env():
    answer = input('fpp conatiner will be removed, type yes to confirm: ')
    if answer != 'yes':
        return

    logging.info('begin reset_env')
    rm_fpp_container()
    logging.info('end reset_env')

def run_perf_script():
    perf_output_dir_suffix = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    perf_output_dir_in_container = f'/opt/maf_planning/tmp/perf/{perf_output_dir_suffix}'
    perf_output_dir_in_host = f'{args.maf_planning_dir}/tmp/perf/{perf_output_dir_suffix}'
    perf_script_path_in_container = '/opt/maf_planning/tools/perf.sh'
    cmd = f'docker exec -i {fpp_container_name} bash -c "{perf_script_path_in_container} {perf_output_dir_in_container}"'
    logging.info('run_perf_script, cmd: %s', cmd)
    os.system(cmd)
    flame_graph_path_in_host = f'{perf_output_dir_in_host}/flamegraph.svg'
    return flame_graph_path_in_host

def get_host_ip():
    h_name = socket.gethostname()
    ip = socket.gethostbyname(h_name)
    return ip

def find_port(ip, port=8000):
    while (True):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            if s.connect_ex((ip, port)) == 0:
                port += 1
                continue
            else:
                return port

def start_http_server_for_flame_graph(flame_graph_path_in_host):
    flame_graph_dir = pathlib.Path(flame_graph_path_in_host).parent
    flame_graph_name = pathlib.Path(flame_graph_path_in_host).name
    ip = get_host_ip()
    port = find_port(ip)
    logging.info('flame graph url: http://%s:%s/%s', ip, port, flame_graph_name)
    os.system(f'python -m http.server {port} --bind {ip} --directory {flame_graph_dir}')

def perf(bag_file_path_in_container):
    logging.info('begin perf')

    proc_play = multiprocessing.Process(target=play_bag, args=(bag_file_path_in_container,))
    proc_play.start()
    flame_graph_path_in_host = run_perf_script()
    proc_play.join()

    if pathlib.Path(flame_graph_path_in_host).exists():
        logging.info('succeed to generate flame graph: %s', flame_graph_path_in_host)
    else:
        logging.error('fail to generate flame graph')
        return

    logging.info('end perf')

    start_http_server_for_flame_graph(flame_graph_path_in_host)


def check_fpp_image_update():
    if not is_container_running(fpp_container_name):
        return
    if args.stop_container:
        stop_container(fpp_container_name)
        sys.exit(1)
    if not is_same_image(args.fpp_image, fpp_container_name):
        answer = input('fpp image changed, container will be stopped and start a new one, type yes to confirm: ')
        if answer == 'yes':
            stop_container(fpp_container_name)
        else:
            sys.exit(1)

check_fpp_image_update()

if args.reset_env:
    reset_env()
elif args.start_container:
    check_to_start_fpp_container()
elif args.stop_container:
    stop_container(fpp_container_name)
elif args.attach:
    attach_fpp_container()
elif args.attach_ros:
    attach_ros_container()
elif args.build:
    check_to_start_fpp_container()
elif args.deploy:
    check_to_start_fpp_container()
elif args.debug:
    check_to_start_fpp_container()
    bag_file_path_in_container = get_bag_file_path_in_container()
    debug(bag_file_path_in_container)
elif args.stress:
    check_to_start_fpp_container()
    bag_file_path_in_container = get_bag_file_path_in_container()
    stress_test(bag_file_path_in_container)
elif args.perf:
    check_to_start_fpp_container()
    bag_file_path_in_container = get_bag_file_path_in_container()
    plan_file_path = perf(bag_file_path_in_container)
elif args.bag or args.mviz:
    check_to_start_fpp_container()
    bag_file_path_in_container = get_bag_file_path_in_container()
    plan_file_path = play_bag(bag_file_path_in_container)
    if args.mviz:
        gen_mviz(plan_file_path)
else:
    attach_fpp_container()
