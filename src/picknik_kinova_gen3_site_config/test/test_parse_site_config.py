# Copyright 2023 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

import unittest
import os
from moveit_studio_utils_py.system_config import SystemConfigParser


class TestParseSiteConfig(unittest.TestCase):
    def test_if_site_config_is_parsed_without_errors(self):
        os.environ["STUDIO_CONFIG_PACKAGE"] = "picknik_004_kinova_gen3_config"

        # Currently, SystemConfigParser calls exit() within its constructor if there is a config parsing error, so if parsing finishes then it also has succeeded.
        system_config_parser = SystemConfigParser()
        self.assertIsNotNone(system_config_parser)
