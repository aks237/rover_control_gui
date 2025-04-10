# process_manager.py
import subprocess

# Dictionary to keep track of running processes.
processes = {}

def start_process(name, command):
    """
    Starts a process for the given command, stores it by name, and returns the PID.
    """
    proc = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    processes[name] = proc
    return proc.pid

def kill_process(name):
    """
    Kills the process associated with the given name.
    Returns True if found and killed, False otherwise.
    """
    if name in processes:
        proc = processes[name]
        proc.kill()
        del processes[name]
        return True
    return False

def list_processes():
    """
    Returns a dictionary mapping process names to their PIDs.
    """
    return {name: proc.pid for name, proc in processes.items()}
