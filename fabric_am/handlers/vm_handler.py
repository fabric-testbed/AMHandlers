#!/usr/bin/env python3
# MIT License
#
# Copyright (c) 2020 FABRIC Testbed
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
#
# Author: Komal Thareja (kthare10@renci.org)
import traceback
from typing import Tuple

from fabric_cf.actor.core.common.constants import Constants
from fabric_cf.actor.core.plugins.handlers.config_token import ConfigToken
from fabric_cf.actor.handlers.handler_base import HandlerBase


class VMHandler(HandlerBase):
    def __init__(self, logger, properties: dict):
        self.logger = logger
        self.properties = properties

    def create(self, unit: ConfigToken, properties: dict) -> Tuple[dict, ConfigToken]:
        result = None
        try:
            self.logger.info(f"Create invoked for unit: {unit} properties: {properties}")

        except Exception as e:
            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_CREATE,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
            self.logger.error(e)
            self.logger.error(traceback.format_exc())
        finally:

            self.logger.info(f"Create completed")
        return result, unit

    def delete(self, unit: ConfigToken, properties: dict) -> Tuple[dict, ConfigToken]:
        result = None
        try:
            self.logger.info(f"Delete invoked for unit: {unit} properties: {properties}")

        except Exception as e:
            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_DELETE,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
            self.logger.error(e)
            self.logger.error(traceback.format_exc())
        finally:

            self.logger.info(f"Delete completed")
        return result, unit

    def modify(self, unit: ConfigToken, properties: dict) -> Tuple[dict, ConfigToken]:
        result = None
        try:
            self.logger.info(f"Modify invoked for unit: {unit} properties: {properties}")

        except Exception as e:
            self.logger.error(e)
            self.logger.error(traceback.format_exc())
            result = {Constants.PROPERTY_TARGET_NAME: Constants.TARGET_MODIFY,
                      Constants.PROPERTY_TARGET_RESULT_CODE: Constants.RESULT_CODE_EXCEPTION,
                      Constants.PROPERTY_ACTION_SEQUENCE_NUMBER: 0}
        finally:
            self.logger.info(f"Modify completed")
        return result, unit
