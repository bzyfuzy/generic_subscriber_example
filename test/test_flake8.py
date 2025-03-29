# Copyright 2025 BzY*FuZy <bzy.fuzy@gmail.com>
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#


from ament_flake8.main import main_with_errors


class TestFlake8:
    def test_flake8(self):
        rc, errors = main_with_errors(argv=['.'])
        assert rc == 0, 'Found code style errors:\n' + '\n'.join(errors)
