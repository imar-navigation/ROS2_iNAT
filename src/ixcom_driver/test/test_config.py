#!/usr/bin/python3

# Copyright 2023 iMAR Navigation GmbH

import pytest
import os
from pathlib import Path
from ixcom_driver.ixcom_driver.mypy.fun import *


class TestConfig:
    @pytest.mark.test_config
    def test_load_wrong_path(self):
        with pytest.raises(Exception):
            load_config('config.json')  # invalid path

    @pytest.mark.test_config
    def test_load_corrupted_file(self):
        with pytest.raises(Exception):
            path = os.getcwd()
            print(f'cwd: {path}')
            file = os.path.join(path, f'test{os.sep}config', 'config_corrupted.json')
            print(f'cwd: {file}')
            load_config(file)  # file with corrupted json structure

    @pytest.mark.test_config
    def test_load_file_with_missing_keys(self):
        with pytest.raises(Exception):
            path = os.getcwd()
            print(f'cwd: {path}')
            file = os.path.join(path, f'test{os.sep}config', 'config_missing_keys.json')
            print(f'cwd: {file}')
            load_config(file)  # file with corrupted json structure

    @pytest.mark.test_config
    def test_load_file_with_too_much_topics(self):
        with pytest.raises(Exception):
            path = os.getcwd()
            print(f'cwd: {path}')
            file = os.path.join(path, f'test{os.sep}config', 'config_too_much_topics.json')
            print(f'cwd: {file}')
            load_config(file)  # file with more than 32 topics

    @pytest.mark.test_config
    def test_load(self):
        path = Path(os.getcwd()).parent.parent.absolute()
        print(f'cwd: {path}')
        file = os.path.join(path, 'src/ixcom_driver/params/config.json')
        print(f'cwd: {file}')
        load_config(file)   # everything is ok

    @pytest.mark.test_config
    def test_load_none(self):
        with pytest.raises(Exception):
            load_config(None)

    @pytest.mark.test_config
    def test_load_empty(self):
        with pytest.raises(Exception):
            load_config('')

    @pytest.mark.test_config
    def test_load_num(self):
        with pytest.raises(Exception):
            load_config(42)

    # @pytest.mark.test_config
    # @pytest.mark.parametrize("f", ["config_corrupted.json",
    #                                "config_missing_keys.json",
    #                                "config_too_much_topics.json"])
    # def test_load_file_with_error(self, f):
    #     with pytest.raises(Exception):
    #         path = os.getcwd()
    #         print(f'cwd: {path}')
    #         file = os.path.join(path, 'test', f)
    #         print(f'cwd: {file}')
    #         load_config(file)

    @pytest.mark.skip
    def test_test(self):
        print('running conf test')
        assert 2 + 2 == 5