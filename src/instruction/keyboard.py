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


class KeyboardInput(object):
    def __init__(self):
        self._object_ids = {
            1: 'hand',
            2: 'remote',
            3: 'cellphone',
            4: 'mouse',
            5: 'bottle',
            6: 'cup',
            7: 'ball'
        }
        self._target_ids = {
            1: 'hand',
            2: 'table'
        }
        s = '\nDemonstration instruction:\n'
        s += 'Take object A (from my hand) and put it on the table (in my hand).\n'
        s += 'Select one object identifier out of\n'
        for k, v in sorted(self._object_ids.items()):
            s += '\t{}\t{}\n'.format(k, v)
        s += 'Select one target identifier out of\n'
        for k, v in sorted(self._target_ids.items()):
            s += '\t{}\t{}\n'.format(k, v)
        s += "To exit the demonstration, input '0'."
        print s

        # print out some integer - object id and integer - target id mapping table
        # take object a and put it on table
        # take object b and give it to me
        # take it from me (my hand) and put it on the table
        # exit
        pass

    def instruct(self):
        # request two ints from the user, returning the corresponding ids in a string
        # separated by a space
        def get_int_in_dict(s, d):
            valid = False
            while not valid:
                id = input(s)
                if not isinstance(id, int):
                    print "Identifier must be an integer. Try again."
                    continue
                if id == 0:
                    return 'exit'
                try:
                    return d[id]
                except KeyError:
                    print "Not a valid identifier. Try again."
                    continue

        oid = get_int_in_dict("Input integer object id: ", self._object_ids)
        if oid == 'exit':
            return oid
        tid = get_int_in_dict("Input integer target id: ", self._target_ids)
        if tid == 'exit':
            return tid
        return '{} {}'.format(oid, tid)


if __name__ == '__main__':
    ki = KeyboardInput()
    instr = ki.instruct()
    print instr
    while instr != 'exit':
        object_id, target_id = instr.split(' ')
        print 'I take {} and {}.'.format('the {}'.format(object_id) if object_id != 'hand' else 'it',
                                         'give it to you' if target_id == 'hand' else 'put it on the table')
        instr = ki.instruct()
        print instr
