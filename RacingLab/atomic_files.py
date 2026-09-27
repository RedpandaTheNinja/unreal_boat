"""Atomic publication with bounded Windows/OneDrive sharing-lock retries."""
import time
def replace_retry(source,target):
    for attempt in range(6):
        try:source.replace(target);return
        except PermissionError:
            if attempt==5:raise
            time.sleep(.01*(attempt+1))
