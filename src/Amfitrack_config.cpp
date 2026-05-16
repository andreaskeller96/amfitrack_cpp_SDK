//-----------------------------------------------------------------------------
//
//                              AMFITECH APS
//
//                          ALL RIGHTS RESERVED
//
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Includes
//-----------------------------------------------------------------------------
#include "Amfitrack_config.h"

#include "Amfitrack_Devices.h"
#include "lib_AmfiProt_API.hpp"
#include "lib_log.h"

#include <algorithm>
#include <cstddef>
#include <cstring>

//-----------------------------------------------------------------------------
// Section: Define
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Typedef
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Section: Macro
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Section: Variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Function prototypes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Functions
//-----------------------------------------------------------------------------

namespace
{
template <typename T, std::size_t N>
constexpr std::size_t array_count(T const (&)[N])
{
	return N;
}

void copy_payload_name(char *dest, std::size_t dest_size, char const *src, std::size_t src_size)
{
	if ((dest == nullptr) || (dest_size == 0U))
	{
		return;
	}
	dest[0] = '\0';

	if (src == nullptr)
	{
		return;
	}

	std::size_t name_length = 0U;
	while ((name_length < src_size) && (src[name_length] != '\0'))
	{
		name_length++;
	}

	const std::size_t copy_length = std::min(name_length, dest_size - 1U);
	if (copy_length > 0U)
	{
		std::memcpy(dest, src, copy_length);
	}
	dest[copy_length] = '\0';
}

void sanitize(DeviceConfig_t &config)
{
	const std::size_t category_capacity = array_count(config.categories);
	if (config.categoryCount > category_capacity)
	{
		LOG_W("sanitize: categoryCount %u exceeds capacity %u, resetting to 0",
			  config.categoryCount,
			  (unsigned)category_capacity);
		config.categoryCount = 0U;
	}

	for (std::size_t category_index = 0U; category_index < category_capacity; category_index++)
	{
		CategoryEntry_t &category = config.categories[category_index];
		if (category.configCount > array_count(category.configs))
		{
			LOG_W("sanitize: category[%u] configCount %u exceeds capacity %u, resetting to 0",
				  (unsigned)category_index,
				  category.configCount,
				  (unsigned)array_count(category.configs));
			category.configCount = 0U;
		}
	}
}

DeviceConfig_t load_config(uint8_t device_id)
{
	DeviceConfig_t config = {};
	AMFITRACK_Sensor sensor;

	if (AMFITRACK_Devices::getInstance().get_sensor(device_id, &sensor))
	{
		config = sensor.config;
		sanitize(config);
		LOG_D("load_config: device_id=%u, categoryCount=%u", device_id, config.categoryCount);
	}
	else
	{
		LOG_W("load_config: device_id=%u not found, returning empty config", device_id);
	}

	return config;
}

CategoryEntry_t *ensure_category(DeviceConfig_t &config, uint8_t category_index)
{
	if (category_index >= array_count(config.categories))
	{
		LOG_E("ensure_category: category_index %u out of bounds (capacity=%u)",
			  category_index,
			  (unsigned)array_count(config.categories));
		return nullptr;
	}

	if (config.categoryCount <= category_index)
	{
		LOG_D("ensure_category: extending categoryCount from %u to %u",
			  config.categoryCount,
			  category_index + 1U);
		config.categoryCount = static_cast<uint8_t>(category_index + 1U);
	}

	CategoryEntry_t &category = config.categories[category_index];
	category.index = category_index;
	return &category;
}

bool store_config(uint8_t device_id, DeviceConfig_t const &config)
{
	const bool stored = AMFITRACK_Devices::getInstance().set(device_id, config);
	if (!stored)
	{
		LOG_E("store_config: failed to store config for device_id=%u", device_id);
	}
	return stored;
}
} // namespace

AMFITRACK_Config &AMFITRACK_Config::getInstance()
{
	static AMFITRACK_Config instance;
	return instance;
}

bool AMFITRACK_Config::start(uint8_t device_id)
{
	if (!AMFITRACK_Devices::is_valid_device_id(device_id))
	{
		LOG_E("start: invalid device_id=%u", device_id);
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	LOG_I("start: beginning config discovery for device_id=%u", device_id);

	DeviceConfig_t config = {};
	store_config(device_id, config);

	_device_id = device_id;
	_state = CONFIG_DISCOVERY_CATEGORY_COUNT;
	_category_index = 0U;
	_config_index = 0U;
	_waiting_for_reply = false;

	return request_current();
}

void AMFITRACK_Config::run()
{
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	request_current();
}

ConfigDiscoveryState_t AMFITRACK_Config::state(uint8_t device_id) const
{
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	if (device_id != _device_id)
	{
		return CONFIG_DISCOVERY_IDLE;
	}

	return _state;
}

bool AMFITRACK_Config::set(uint8_t device_id, lib_AmfiProt_ConfigCategoryCount_t const &category_count)
{
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	LOG_D("set(CategoryCount): device_id=%u, categoryCount=%u", device_id, category_count.categoryCount);

	DeviceConfig_t config = load_config(device_id);

	const std::size_t category_capacity = array_count(config.categories);
	const uint8_t count = static_cast<uint8_t>(std::min<std::size_t>(category_count.categoryCount, category_capacity));
	const uint8_t previous_count = config.categoryCount;

	if (category_count.categoryCount > category_capacity)
	{
		LOG_W("set(CategoryCount): requested count %u exceeds capacity %u, clamping to %u",
			  category_count.categoryCount,
			  (unsigned)category_capacity,
			  count);
	}

	if (count < previous_count)
	{
		LOG_D("set(CategoryCount): shrinking from %u to %u categories", previous_count, count);
		for (std::size_t category_index = count; category_index < previous_count; category_index++)
		{
			std::memset(&config.categories[category_index], 0, sizeof(config.categories[category_index]));
		}
	}

	for (std::size_t category_index = previous_count; category_index < count; category_index++)
	{
		config.categories[category_index].index = static_cast<uint8_t>(category_index);
	}

	config.categoryCount = count;
	const bool stored = store_config(device_id, config);

	if (stored && is_active(device_id, CONFIG_DISCOVERY_CATEGORY_COUNT))
	{
		LOG_I("set(CategoryCount): advancing to CATEGORY_NAMES, count=%u", count);
		_state = CONFIG_DISCOVERY_CATEGORY_NAMES;
		_category_index = 0U;
		_config_index = 0U;
		_waiting_for_reply = false;

		if (config.categoryCount == 0U)
		{
			LOG_I("set(CategoryCount): no categories, finishing early");
			finish();
		}
		else
		{
			request_current();
		}
	}

	return stored;
}

bool AMFITRACK_Config::set(uint8_t device_id, lib_AmfiProt_ConfigCategory_t const &category)
{
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	LOG_D("set(Category): device_id=%u, categoryIndex=%u, name=\"%.*s\"",
		  device_id,
		  category.categoryIndex,
		  (int)sizeof(category.name),
		  category.name);

	DeviceConfig_t config = load_config(device_id);
	CategoryEntry_t *category_entry = ensure_category(config, category.categoryIndex);

	if (category_entry == nullptr)
	{
		LOG_E("set(Category): failed to ensure category at index %u", category.categoryIndex);
		return false;
	}

	copy_payload_name(category_entry->name, sizeof(category_entry->name), category.name, sizeof(category.name));
	const bool stored = store_config(device_id, config);

	if (stored && is_active(device_id, CONFIG_DISCOVERY_CATEGORY_NAMES) && (category.categoryIndex == _category_index))
	{
		_category_index++;
		_waiting_for_reply = false;
		LOG_D("set(Category): category name stored, advancing to category_index=%u", _category_index);

		if (_category_index >= config.categoryCount)
		{
			LOG_I("set(Category): all category names received, advancing to VALUE_COUNT");
			advance_to_value_count(config);
		}

		request_current();
	}

	return stored;
}

bool AMFITRACK_Config::set(uint8_t device_id, lib_AmfiProt_ConfigValueCount_t const &config_count)
{
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	LOG_D("set(ValueCount): device_id=%u, category=%u, count=%u",
		  device_id,
		  config_count.category,
		  config_count.count);

	DeviceConfig_t config = load_config(device_id);
	CategoryEntry_t *category = ensure_category(config, config_count.category);

	if (category == nullptr)
	{
		LOG_E("set(ValueCount): failed to ensure category at index %u", config_count.category);
		return false;
	}

	const std::size_t config_capacity = array_count(category->configs);
	const uint8_t count = static_cast<uint8_t>(std::min<std::size_t>(config_count.count, config_capacity));
	const uint8_t previous_count = category->configCount;

	if (config_count.count > config_capacity)
	{
		LOG_W("set(ValueCount): requested count %u exceeds capacity %u for category %u, clamping to %u",
			  config_count.count,
			  (unsigned)config_capacity,
			  config_count.category,
			  count);
	}

	if (count < previous_count)
	{
		LOG_D("set(ValueCount): shrinking category %u from %u to %u configs",
			  config_count.category,
			  previous_count,
			  count);
		for (std::size_t config_index = count; config_index < previous_count; config_index++)
		{
			std::memset(&category->configs[config_index], 0, sizeof(category->configs[config_index]));
		}
	}

	for (std::size_t config_index = previous_count; config_index < count; config_index++)
	{
		category->configs[config_index].categoryIndex = category->index;
	}

	category->configCount = count;
	const bool stored = store_config(device_id, config);

	if (stored && is_active(device_id, CONFIG_DISCOVERY_VALUE_COUNT) && (config_count.category == _category_index))
	{
		_category_index++;
		_waiting_for_reply = false;
		LOG_D("set(ValueCount): value count stored, advancing to category_index=%u", _category_index);

		if (_category_index >= config.categoryCount)
		{
			LOG_I("set(ValueCount): all value counts received, advancing to NAMES_BY_UID");
			advance_to_names_by_uid(config);
		}

		request_current();
	}

	return stored;
}

bool AMFITRACK_Config::set(uint8_t device_id, lib_AmfiProt_ConfigNameUID_protocol_t const &config_name)
{
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	const uint8_t category_index = config_name.config_names.category;
	const uint16_t config_index = config_name.index;

	LOG_D("set(NameUID): device_id=%u, category=%u, index=%u, uid=%u, name=\"%.*s\"",
		  device_id,
		  category_index,
		  config_index,
		  config_name.config_names.uid,
		  (int)sizeof(config_name.config_names.name),
		  config_name.config_names.name);

	DeviceConfig_t config = load_config(device_id);
	CategoryEntry_t *category = ensure_category(config, category_index);

	if ((category == nullptr) || (config_index >= array_count(category->configs)))
	{
		LOG_E("set(NameUID): invalid category %u or config_index %u out of bounds",
			  category_index,
			  config_index);
		return false;
	}

	if (category->configCount <= config_index)
	{
		category->configCount = static_cast<uint8_t>(config_index + 1U);
	}

	ConfigEntry_t &config_entry = category->configs[config_index];
	config_entry.uid = config_name.config_names.uid;
	config_entry.categoryIndex = category_index;
	copy_payload_name(config_entry.name, sizeof(config_entry.name), config_name.config_names.name, sizeof(config_name.config_names.name));

	const bool stored = store_config(device_id, config);

	if (stored && is_active(device_id, CONFIG_DISCOVERY_NAMES_BY_UID) &&
		(category_index == _category_index) &&
		(config_index == _config_index))
	{
		_config_index++;
		_waiting_for_reply = false;
		LOG_D("set(NameUID): name stored, advancing to config_index=%u", _config_index);

		if (!select_next_config(config))
		{
			LOG_I("set(NameUID): all names by UID received, advancing to VALUES_BY_UID");
			advance_to_values_by_uid(config);
		}

		request_current();
	}

	return stored;
}

bool AMFITRACK_Config::set(uint8_t device_id, lib_AmfiProt_ConfigValueUID_t const &config_value)
{
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	LOG_D("set(ValueUID): device_id=%u, uid=%u", device_id, config_value.uid);

	DeviceConfig_t config = load_config(device_id);
	bool found = false;

	for (std::size_t category_index = 0U; category_index < config.categoryCount; category_index++)
	{
		CategoryEntry_t &category = config.categories[category_index];

		for (std::size_t config_index = 0U; config_index < category.configCount; config_index++)
		{
			ConfigEntry_t &config_entry = category.configs[config_index];
			if (config_entry.uid == config_value.uid)
			{
				LOG_D("set(ValueUID): matched uid=%u at category=%u, index=%u",
					  config_value.uid,
					  (unsigned)category_index,
					  (unsigned)config_index);
				config_entry.config = config_value.value;
				found = true;
			}
		}
	}

	if (!found)
	{
		LOG_W("set(ValueUID): uid=%u not found in any category for device_id=%u",
			  config_value.uid,
			  device_id);
		return false;
	}

	const bool stored = store_config(device_id, config);

	if (stored && is_active(device_id, CONFIG_DISCOVERY_VALUES_BY_UID) &&
		(_category_index < config.categoryCount) &&
		(_config_index < config.categories[_category_index].configCount) &&
		(config.categories[_category_index].configs[_config_index].uid == config_value.uid))
	{
		_config_index++;
		_waiting_for_reply = false;
		LOG_D("set(ValueUID): value stored, advancing to config_index=%u", _config_index);

		if (!select_next_config(config))
		{
			LOG_I("set(ValueUID): all values by UID received, discovery complete");
			finish();
		}

		request_current();
	}

	return stored;
}

bool AMFITRACK_Config::request_current()
{
	if (_waiting_for_reply)
	{
		LOG_D("request_current: waiting for reply, skipping (state=%d)", (int)_state);
		return true;
	}

	switch (_state)
	{
		case CONFIG_DISCOVERY_CATEGORY_COUNT:
			return request_category_count();
		case CONFIG_DISCOVERY_CATEGORY_NAMES:
			return request_category_name();
		case CONFIG_DISCOVERY_VALUE_COUNT:
			return request_value_count();
		case CONFIG_DISCOVERY_NAMES_BY_UID:
			return request_name_by_uid();
		case CONFIG_DISCOVERY_VALUES_BY_UID:
			return request_value_by_uid();
		case CONFIG_DISCOVERY_IDLE:
		case CONFIG_DISCOVERY_DONE:
		default:
			return true;
	}
}

bool AMFITRACK_Config::request_category_count()
{
	LOG_D("request_category_count: device_id=%u", _device_id);
	uint8_t payload = static_cast<uint8_t>(lib_AmfiProt_PayloadID_RequestCategoryCount);
	const bool queued = AmfiProt_API::getInstance().queue_frame(&payload, sizeof(payload), libAmfiProt_PayloadType_Common, lib_AmfiProt_packetType_NoAck, _device_id);
	if (!queued)
	{
		LOG_W("request_category_count: failed to queue frame for device_id=%u", _device_id);
	}
	_waiting_for_reply = queued;
	return queued;
}

bool AMFITRACK_Config::request_category_name()
{
	DeviceConfig_t config = load_config(_device_id);
	if (_category_index >= config.categoryCount)
	{
		LOG_D("request_category_name: category_index=%u >= categoryCount=%u, advancing",
			  _category_index,
			  config.categoryCount);
		advance_to_value_count(config);
		return request_current();
	}

	LOG_D("request_category_name: requesting category_index=%u", _category_index);

	lib_AmfiProt_ConfigCategoryRequest_t payload = {};
	payload.payloadID = lib_AmfiProt_PayloadID_RequestConfigurationCategory;
	payload.categoryIndex = _category_index;

	const bool queued = AmfiProt_API::getInstance().queue_frame(&payload, sizeof(payload), libAmfiProt_PayloadType_Common, lib_AmfiProt_packetType_NoAck, _device_id);
	if (!queued)
	{
		LOG_W("request_category_name: failed to queue frame for category_index=%u", _category_index);
	}
	_waiting_for_reply = queued;
	return queued;
}

bool AMFITRACK_Config::request_value_count()
{
	DeviceConfig_t config = load_config(_device_id);
	if (_category_index >= config.categoryCount)
	{
		LOG_D("request_value_count: category_index=%u >= categoryCount=%u, advancing",
			  _category_index,
			  config.categoryCount);
		advance_to_names_by_uid(config);
		return request_current();
	}

	LOG_D("request_value_count: requesting value count for category_index=%u", _category_index);

	lib_AmfiProt_ConfigValueCountRequest_t payload = {};
	payload.payloadID = lib_AmfiProt_PayloadID_RequestConfigurationValueCount;
	payload.category = _category_index;

	const bool queued = AmfiProt_API::getInstance().queue_frame(&payload, sizeof(payload), libAmfiProt_PayloadType_Common, lib_AmfiProt_packetType_NoAck, _device_id);
	if (!queued)
	{
		LOG_W("request_value_count: failed to queue frame for category_index=%u", _category_index);
	}
	_waiting_for_reply = queued;
	return queued;
}

bool AMFITRACK_Config::request_name_by_uid()
{
	DeviceConfig_t config = load_config(_device_id);
	if (!select_next_config(config))
	{
		LOG_D("request_name_by_uid: no more configs, advancing to VALUES_BY_UID");
		advance_to_values_by_uid(config);
		return request_current();
	}

	LOG_D("request_name_by_uid: requesting category=%u, index=%u", _category_index, _config_index);

	lib_AmfiProt_ConfigNameRequestUID_t payload = {};
	payload.payloadID = lib_AmfiProt_PayloadID_RequestConfigurationNameAndUID;
	payload.category = _category_index;
	payload.index = _config_index;

	const bool queued = AmfiProt_API::getInstance().queue_frame(&payload, sizeof(payload), libAmfiProt_PayloadType_Common, lib_AmfiProt_packetType_NoAck, _device_id);
	if (!queued)
	{
		LOG_W("request_name_by_uid: failed to queue frame for category=%u, index=%u",
			  _category_index,
			  _config_index);
	}
	_waiting_for_reply = queued;
	return queued;
}

bool AMFITRACK_Config::request_value_by_uid()
{
	DeviceConfig_t config = load_config(_device_id);
	if (!select_next_config(config))
	{
		LOG_D("request_value_by_uid: no more configs, finishing");
		finish();
		return true;
	}

	const uint32_t uid = config.categories[_category_index].configs[_config_index].uid;
	LOG_D("request_value_by_uid: requesting uid=%u (category=%u, index=%u)",
		  uid,
		  _category_index,
		  _config_index);

	lib_AmfiProt_ConfigValueUIDRequest_t payload = {};
	payload.payloadID = lib_AmfiProt_PayloadID_RequestConfigurationValueUID;
	payload.uid = uid;

	const bool queued = AmfiProt_API::getInstance().queue_frame(&payload, sizeof(payload), libAmfiProt_PayloadType_Common, lib_AmfiProt_packetType_NoAck, _device_id);
	if (!queued)
	{
		LOG_W("request_value_by_uid: failed to queue frame for uid=%u", uid);
	}
	_waiting_for_reply = queued;
	return queued;
}

bool AMFITRACK_Config::select_next_config(DeviceConfig_t const &config)
{
	while (_category_index < config.categoryCount)
	{
		if (_config_index < config.categories[_category_index].configCount)
		{
			return true;
		}

		_category_index++;
		_config_index = 0U;
	}

	return false;
}

bool AMFITRACK_Config::is_active(uint8_t device_id, ConfigDiscoveryState_t state) const
{
	return (device_id == _device_id) && (_state == state);
}

void AMFITRACK_Config::advance_to_value_count(DeviceConfig_t const &config)
{
	(void)config;
	LOG_I("advance_to_value_count: entering VALUE_COUNT state");
	_state = CONFIG_DISCOVERY_VALUE_COUNT;
	_category_index = 0U;
	_config_index = 0U;
	_waiting_for_reply = false;
}

void AMFITRACK_Config::advance_to_names_by_uid(DeviceConfig_t const &config)
{
	LOG_I("advance_to_names_by_uid: entering NAMES_BY_UID state");
	_state = CONFIG_DISCOVERY_NAMES_BY_UID;
	_category_index = 0U;
	_config_index = 0U;
	_waiting_for_reply = false;

	if (!select_next_config(config))
	{
		LOG_I("advance_to_names_by_uid: no configs found, skipping to VALUES_BY_UID");
		advance_to_values_by_uid(config);
	}
}

void AMFITRACK_Config::advance_to_values_by_uid(DeviceConfig_t const &config)
{
	LOG_I("advance_to_values_by_uid: entering VALUES_BY_UID state");
	_state = CONFIG_DISCOVERY_VALUES_BY_UID;
	_category_index = 0U;
	_config_index = 0U;
	_waiting_for_reply = false;

	if (!select_next_config(config))
	{
		LOG_I("advance_to_values_by_uid: no configs found, finishing");
		finish();
	}
}

static void log_parameter_value(const lib_Generic_Parameter_Value &value)
{
	switch (static_cast<lib_Generic_Parameter_Type_t>(value.type))
	{
		case lib_Generic_Parameter_Type_void:
			LOG_I("        value=(void)");
			break;
		case lib_Generic_Parameter_Type_bool:
			LOG_I("        value=(bool)%s", value.b ? "true" : "false");
			break;
		case lib_Generic_Parameter_Type_char:
			LOG_I("        value=(char)'%c'", value.ch);
			break;
		case lib_Generic_Parameter_Type_s8:
			LOG_I("        value=(int8)%d", (int)value.s8);
			break;
		case lib_Generic_Parameter_Type_u8:
			LOG_I("        value=(uint8)%u", (unsigned)value.u8);
			break;
		case lib_Generic_Parameter_Type_s16LE:
		case lib_Generic_Parameter_Type_s16BE:
			LOG_I("        value=(int16%s)%d",
				  value.type == lib_Generic_Parameter_Type_s16LE ? "LE" : "BE",
				  (int)value.s16);
			break;
		case lib_Generic_Parameter_Type_u16LE:
		case lib_Generic_Parameter_Type_u16BE:
			LOG_I("        value=(uint16%s)%u",
				  value.type == lib_Generic_Parameter_Type_u16LE ? "LE" : "BE",
				  (unsigned)value.u16);
			break;
		case lib_Generic_Parameter_Type_s32LE:
		case lib_Generic_Parameter_Type_s32BE:
			LOG_I("        value=(int32%s)%ld",
				  value.type == lib_Generic_Parameter_Type_s32LE ? "LE" : "BE",
				  (long)value.s32);
			break;
		case lib_Generic_Parameter_Type_u32LE:
		case lib_Generic_Parameter_Type_u32BE:
			LOG_I("        value=(uint32%s)%lu",
				  value.type == lib_Generic_Parameter_Type_u32LE ? "LE" : "BE",
				  (unsigned long)value.u32);
			break;
		case lib_Generic_Parameter_Type_s64LE:
		case lib_Generic_Parameter_Type_s64BE:
			LOG_I("        value=(int64%s)%lld",
				  value.type == lib_Generic_Parameter_Type_s64LE ? "LE" : "BE",
				  (long long)value.s64);
			break;
		case lib_Generic_Parameter_Type_u64LE:
		case lib_Generic_Parameter_Type_u64BE:
			LOG_I("        value=(uint64%s)%llu",
				  value.type == lib_Generic_Parameter_Type_u64LE ? "LE" : "BE",
				  (unsigned long long)value.u64);
			break;
		case lib_Generic_Parameter_Type_f32LE:
		case lib_Generic_Parameter_Type_f32BE:
			LOG_I("        value=(float%s)%f",
				  value.type == lib_Generic_Parameter_Type_f32LE ? "LE" : "BE",
				  (double)value.f32);
			break;
		case lib_Generic_Parameter_Type_f64LE:
		case lib_Generic_Parameter_Type_f64BE:
			LOG_I("        value=(double%s)%f",
				  value.type == lib_Generic_Parameter_Type_f64LE ? "LE" : "BE",
				  value.f64);
			break;
		case lib_Generic_Parameter_Type_ProcedureCall:
			LOG_I("        value=(ProcedureCall)");
			break;
		default:
			LOG_W("        value=(unknown type=%u)", value.type);
			break;
	}
}

void AMFITRACK_Config::print_config(uint8_t device_id)
{
	DeviceConfig_t config = load_config(device_id);

	LOG_I("==============================");
	LOG_I("Config dump for device_id=%u", device_id);
	LOG_I("  categoryCount=%u", config.categoryCount);

	for (std::size_t category_index = 0U; category_index < config.categoryCount; category_index++)
	{
		const CategoryEntry_t &category = config.categories[category_index];
		LOG_I("  [Category %u] \"%s\" (%u config(s))",
			  category.index,
			  category.name,
			  category.configCount);

		for (std::size_t config_index = 0U; config_index < category.configCount; config_index++)
		{
			const ConfigEntry_t &entry = category.configs[config_index];
			LOG_I("    [%u] \"%s\" (uid=%u)", (unsigned)config_index, entry.name, entry.uid);
			log_parameter_value(entry.config);
		}
	}

	LOG_I("==============================");
}

void AMFITRACK_Config::finish()
{
	LOG_I("finish: config discovery complete for device_id=%u", _device_id);
	_state = CONFIG_DISCOVERY_DONE;
	_category_index = 0U;
	_config_index = 0U;
	_waiting_for_reply = false;
}