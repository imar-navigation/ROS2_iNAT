#!/usr/bin/env python3

import subprocess
import time
import sys
import os
import operator


def log(msg):
    print(f'private_filter: {msg}')


def __filter_xcomdat(f_in, f_out):
    log('filtering header')
    log(f'input file: {f_in}')
    log(f'output file: {f_out}')

    try:
        f = open(f_in, 'r')
    except:
        log(f'could not open {f_in}')
        sys.exit(1)
    lns = f.readlines()
    f.close()

    def2move = ''
    private_entries = []
    i = 0
    for ln in lns:
        if '#domain:' in ln.lower() and 'private' in ln.lower():
            private_entry = {}
            private_entry['pos'] = i

            for j in range(i, -1, -1):
                if '#name' in lns[j].lower():
                    private_entry['name'] = lns[j].split('#name:')[1].strip(' ').strip('\n')
                    log(f'private entry found: {private_entry["name"]}')
                if '/**' in lns[j]:
                    private_entry['start_pos'] = j
                    break

            for j in range(i, i + 100, 1):
                if '#name' in lns[j].lower():
                    private_entry['name'] = lns[j].split('#name:')[1].strip(' ').strip('\n')
                    log(f'private entry found: {private_entry["name"]}')
                if '*/' in lns[j]:
                    private_entry['comment_end_pos'] = j
                    break

            content_pos = private_entry['comment_end_pos'] + 1
            if private_entry.get('name'):
                for j in range(content_pos, content_pos + 1000, 1):
                    if '#define XCOMPAR_PLUGIN_MAX_NUM' in lns[j]:
                        def2move = lns[j]
                    if private_entry['name'] in lns[j]:
                        private_entry['end_pos'] = j
                        break
            else:
                c = 0
                content_found = False
                for j in range(content_pos, content_pos + 1000, 1):  # could fail if no braces found
                    if '#define XCOMPAR_PLUGIN_MAX_NUM' in lns[j]:
                        def2move = lns[j]
                    if '{' in lns[j]:
                        c += 1
                        content_found = True
                    if '}' in lns[j]:
                        c -= 1
                    if content_found and c == 0:
                        private_entry['end_pos'] = j

                        if not private_entry.get('name'):
                            private_entry['name'] = lns[j].replace('}', '').replace(';', '').strip()
                            log(f'private entry found: {private_entry["name"]}')

                        break

            private_entries.append(private_entry)
        i += 1

    if len(private_entries) == 0:
        log('no private content found')
        sys.exit(0)

    for i in range(len(private_entries) - 1, -1, -1):
        del lns[private_entries[i]['start_pos']:private_entries[i]['end_pos'] + 2]

    if not def2move == '':
        general_found = False
        for i in range(len(lns)):
            if '* General' in lns[i]:
                general_found = True
            if general_found:
                ln = lns[i].strip('\n')
                # print(f'line: {ln}, len: {len(ln)}')
                if len(ln) == 0:
                    lns[i] = def2move
                    break

    try:
        f = open(f_out, 'w+')
    except:
        log(f'could not open {f_out}')
        sys.exit(1)
    f.writelines(lns)
    f.close()

    private_entries.sort(key=operator.itemgetter('start_pos'))

    return private_entries

    # print(private_entries)
    # sys.exit(1)


def __filter_traits(f_in, f_out, private_entries_hdr):
    log('filtering traits')
    log(f'input file: {f_in}')
    log(f'output file: {f_out}')

    try:
        f = open(f_in, 'r')
    except:
        log(f'could not open {f_in}')
        sys.exit(1)
    lns = f.readlines()
    f.close()

    private_entries = []
    for private_entry_hdr in private_entries_hdr:
        i = 0
        for ln in lns:
            if private_entry_hdr['name'] in ln:
                private_entry = {'name': private_entry_hdr['name']}
                for j in range(i, -1, -1):
                    if 'template' in lns[j]:
                        private_entry['start_pos'] = j
                        break

                c = 0
                content_found = False
                for j in range(i, i + 10, 1):
                    if '{' in lns[j]:
                        c += 1
                        content_found = True
                    if '}' in lns[j]:
                        c -= 1
                    if content_found and c == 0:
                        private_entry['end_pos'] = j
                        break
                private_entries.append(private_entry)
                break
            i += 1

    private_entries.sort(key=operator.itemgetter('start_pos'))
    print(private_entries)
    # sys.exit(1)

    for i in range(len(private_entries) - 1, -1, -1):
        del lns[private_entries[i]['start_pos']:private_entries[i]['end_pos'] + 2]

    try:
        f = open(f_out, 'w+')
    except:
        log(f'could not open {f_out}')
        sys.exit(1)
    f.writelines(lns)
    f.close()


def __cleanup(f_orig, f_mod):

    if not os.path.exists(f_orig):
        log(f'could not open {f_orig}')
        sys.exit(1)

    if not os.path.exists(f_mod):
        log(f'could not open {f_mod}')
        sys.exit(1)

    # try:
    #     f_orig_ = open(f_orig, 'r')
    # except:
    #     log(f'could not open {f_orig}')
    #     sys.exit(1)
    # try:
    #     f_mod_ = open(f_mod, 'r')
    # except:
    #     log(f'could not open {f_mod}')
    #     sys.exit(1)
    #
    # if not f_orig_.writable():
    #     log(f'file {f_orig} is not writable')
    #     sys.exit(1)
    #
    # if not f_mod_.writable():
    #     log(f'file {f_mod} is not writable')
    #     sys.exit(1)

    try:
        os.remove(f_orig)
    except:
        log(f'failed to remove original file {f_orig}')
        sys.exit(1)

    try:
        os.rename(f_mod, f_orig)
    except:
        log(f'failed to rename modified file {f_orig}')
        sys.exit(1)


def main():
    file_dat_in = 'include/ixcom/XCOMdat.h'
    file_dat_out = 'include/ixcom/XCOMdat_filtered.h'
    private_entries = __filter_xcomdat(file_dat_in, file_dat_out)

    file_par_in = 'src/parameter_traits.h'
    file_par_out = 'src/parameter_traits_filtered.h'
    __filter_traits(file_par_in, file_par_out, private_entries)

    file_msg_in = 'src/message_traits.h'
    file_msg_out = 'src/message_traits_filtered.h'
    __filter_traits(file_msg_in, file_msg_out, private_entries)

    __cleanup(file_dat_in, file_dat_out)
    __cleanup(file_par_in, file_par_out)
    __cleanup(file_msg_in, file_msg_out)

    sys.exit(0)


if __name__ == '__main__':
    main()
