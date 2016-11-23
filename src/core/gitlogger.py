# Copyright (c) 2016, BRML
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# This versioning logger was inspired by
#   https://github.com/surban/mlutils/commit/1770edb54bdb04b90970705c1

import importlib
import json
import os
from subprocess import Popen, PIPE


def git_logger(modules=None, filename=None):
    """Versioning logger that logs the current state of the requested modules.
    If module is provided as a git repository, this state is represented by
    the latest git commit.
    If module is not provided as a git repository, the version info of the
    module is stored, if available.

    Usage: print git_logger(['numpy', 'theano'])
           {
                "commits": {
                    "theano": "0693ce052725a15b502068a1490b0637216feb00",
                    "numpy": "1.8.2"
                },
                "warnings": {
                    "theano": ""
                }
            }

    :param modules: A list of modules to check for version information.
    :param filename: If given, the filename to save the log with.
    :return: The created log as a dictionary.
    """
    if modules is None:
        modules = list()
    if not isinstance(modules, list):
        modules = [modules, ]
    current_path = os.getcwd()
    commits = dict()
    warnings = dict()

    for module in modules:
        # get install path of module and go there
        if module.startswith('ros_'):
            import rospkg
            module = module.split('_', 1)[1]
            mod_path = rospkg.RosPack().get_path(module)
        else:
            mod = importlib.import_module(module)
            mod_path = os.path.dirname(mod.__file__)
        os.chdir(mod_path)

        proc = Popen(['git', 'log', '-1', '--pretty=oneline'],
                     stdout=PIPE, stderr=PIPE)
        out, err = proc.communicate()
        if err.startswith('fatal'):
            # mod_path is not a git repository.
            # Fallback to module version instead.
            if hasattr(mod, '__version__'):
                commits[module] = mod.__version__
            else:
                # mod_path is neither a git repository
                # nor has module a version info => give up
                commits[module] = 'no commit or version information found!'
        else:
            commits[module] = out.split(' ')[0].strip()
            # give a warning if there have been changes since latest commit
            proc = Popen(['git', 'diff', '--shortstat'], stdout=PIPE)
            out, _ = proc.communicate()
            if out:
                warnings[module] = out
                warn = "WARNING: uncommitted changes in module '{}'!".format(module)
                print "\033[1;33m%s\033[0m\n" % warn
    logs = {'commits': commits, 'warnings': warnings}
    os.chdir(current_path)

    # save versioning log, if requested
    if isinstance(filename, str) and len(filename) > 0:
        _, ext = os.path.splitext(filename)
        if not ext:
            filename += '.json'
        with open(filename, 'wb') as fp:
            json.dump(logs, fp, indent=4)
    return logs
