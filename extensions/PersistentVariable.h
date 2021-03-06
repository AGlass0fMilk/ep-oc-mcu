/**
 * ep-oc-mcu
 * Embedded Planet Open Core for Microcontrollers
 *
 * Built with ARM Mbed-OS
 *
 * Copyright (c) 2019 Embedded Planet, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef EP_OC_MCU_EXTENSIONS_PERSISTENTVARIABLE_H_
#define EP_OC_MCU_EXTENSIONS_PERSISTENTVARIABLE_H_

#include "kvstore_global_api.h"
#include "platform/mbed_error.h"
#include "platform/mbed_assert.h"

#include <cstring>

#include <stdio.h>

/** Macro fun :) */
#define FORMAT_PARTITION_NAME(s) "/" #s "/"
#define KV_STORE_DEFAULT_PARTITION_NAME(s) FORMAT_PARTITION_NAME(s)

#ifndef COMPONENT_FLASHIAP
#warning PersistentVariable - Persistence will be unavailable unless COMPONENT_FLASHIAP is enabled
#endif

namespace ep
{

	/**
	 * Templatized persistent variable built on top of mbed's
	 * KVStore API. If KVStore is not available (ie: COMPONENT_FLASHIAP is disabled)
	 * the API will fall back to non-volatile storage with a default initialized
	 */
	template<typename T>
	class PersistentVariable
	{

	public:

		/** Initialize a persistent variable with a default value
		 * @param[in] default_value The default value of this persistent variable
		 *
		 * @note This value is only used if the persistent variable has not
		 * been accessed before or if the kvstore is unavailable for some reason
		 */
		PersistentVariable(T default_value, const char* key) : _value(default_value) {

#ifdef COMPONENT_FLASHIAP


			// Default partition name length (configured with json, defaults to '/kv/'
			size_t default_partition_len = strlen(KV_STORE_DEFAULT_PARTITION_NAME(MBED_CONF_STORAGE_DEFAULT_KV));

			// Format the given key to comply with KVStore requirements
			_key = new char[(strlen(key)-1)+default_partition_len];
			strcpy(_key, KV_STORE_DEFAULT_PARTITION_NAME(MBED_CONF_STORAGE_DEFAULT_KV));
			strcpy(&_key[default_partition_len], &key[1]); // skip over the preceding '/'

			// Get the index of the slash between module and variable name
			char* slash = strrchr(_key, '/');

			// Replace it with a dash ('-') to comply with mbed's KVStore requirements
			*slash = '-';

#endif
		}

		/** Destructor */
		~PersistentVariable(void) {

#ifdef COMPONENT_FLASHIAP

			if(_key) {
				delete[] _key;
				_key = NULL;
			}
#endif
		}



		/**
		 * Assignment operator for updating the value stored in persistent
		 * memory (if available)
		 */
		T &operator= (const T &rhs) {
			this->set(rhs);
			return _value;
		}

		/** Evaluation operator
		 *
		 * Reads underlying persistent memory and returns current value (or default)
		 */
		operator T() {
			this->get();
			return _value;
		}

		/** Attempts to get the underlying value from KVStore
		 * @retval value Value obtained from KVStore of default if unavailable */
		T get(void) {

#ifdef COMPONENT_FLASHIAP

			// Try to access the KVStore partition
			size_t actual_size;
			int err = kv_get(_key, &_value, sizeof(T), &actual_size);

			/** If we weren't able to get the variable,
				attempt to set the default value in KVStore */
			if(err == MBED_ERROR_ITEM_NOT_FOUND) {

				this->set(_value);

				// Now try to get the key... if this doesn't work we will return default
				kv_get(_key, &_value, sizeof(T), &actual_size);
			}

#endif

			return _value;
		}

		/** Attempts to set the underlying value in KVStore
		 * @param[in] value Value to set
		 */
		void set(T new_value) {

			_value = new_value;

#ifdef COMPONENT_FLASHIAP

			// Try to access the KVStore partition
			int err = kv_set(_key, &_value, sizeof(T), 0);

			/** If we weren't able to set the variable,
				attempt to initialize the partition */
			if(err != MBED_SUCCESS) {

				// Return the last cached value (most likely default) if this fails
				if(init_kvstore_partition() != MBED_SUCCESS) {
					return;
				} else {
					// Now try to set the key... if this doesn't work value stays default
					kv_set(_key, &_value, sizeof(T), 0);
				}
			}
#endif

		}

		/*
		 * Attempts to initialize the KVStore partition
		 *
		 * @retval err kv store error
		 */
		int init_kvstore_partition(void) {
#ifdef COMPONENT_FLASHIAP

			// Reset the partition with the default name
			return kv_reset(KV_STORE_DEFAULT_PARTITION_NAME(MBED_CONF_STORAGE_DEFAULT_KV));
#endif
		}

	protected:

		T _value;
		char* _key;

	};

}


#endif /* EP_OC_MCU_EXTENSIONS_PERSISTENTVARIABLE_H_ */
