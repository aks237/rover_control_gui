# process_manager.py
import subprocess

processes = {}

def start_process(name, command):
    proc = subprocess.Popen(
        command,
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    processes[name] = proc

    try:
        stdout, stderr = proc.communicate(timeout=1.0)
    except subprocess.TimeoutExpired:
        stdout, stderr = "", ""

    return {
        'pid': proc.pid,
        'stdout': stdout,
        'stderr': stderr,
        'running': proc.poll() is None
    }

def kill_process(name):
    if name in processes:
        proc = processes[name]
        try:
            proc.kill()
            stdout, stderr = proc.communicate(timeout=1.0)
        except subprocess.TimeoutExpired:
            stdout, stderr = "", ""
        del processes[name]
        return True, {'stdout': stdout, 'stderr': stderr}
    return False, None
