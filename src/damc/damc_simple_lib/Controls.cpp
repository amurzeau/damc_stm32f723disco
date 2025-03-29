#include "Controls.h"
#include "usbd_audio.h"
#include <OscRoot.h>
#include <atomic>

Controls* Controls::instance;

Controls::Controls(OscRoot* oscRoot) : oscRoot(oscRoot), processUsbControlChange(false) {
	uv_async_init(uv_default_loop(), &asyncControlChanged, Controls::onControlChangedStatic);
	asyncControlChanged.data = this;
}

void Controls::init() {
	static const char* controlNodeAddresses[] = {
	    "strip/0",
	    "strip/2",
	    "strip/1",
	};

	static_assert(sizeof(controlNodeAddresses) / sizeof(controlNodeAddresses[0]) ==
	                  std::tuple_size<decltype(controlsMapping)>::value,
	              "bad controlsMapping size");

	for(size_t i = 0; i < controlsMapping.size(); i++) {
		OscNode* baseNode = oscRoot->getNode(controlNodeAddresses[i]);
		controlsMapping[i].volumeControl = (OscReadOnlyVariable<float>*) baseNode->getNode("filterChain/volume");
		controlsMapping[i].muteControl = (OscReadOnlyVariable<bool>*) baseNode->getNode("filterChain/mute");
		controlsMapping[i].volumeToSet.isChanged = false;

		controlsMapping[i].volumeControl->addChangeCallback([this, i](float) {
			if(!processUsbControlChange)
				USBD_AUDIO_NotifyUnitIdChanged(i * AUDIO_UNIT_ID_PER_ENDPOINT + AUDIO_UNIT_ID_OFFSET_FEATURE_UNIT);
		});
		controlsMapping[i].muteControl->addChangeCallback([this, i](bool) {
			if(!processUsbControlChange)
				USBD_AUDIO_NotifyUnitIdChanged(i * AUDIO_UNIT_ID_PER_ENDPOINT + AUDIO_UNIT_ID_OFFSET_FEATURE_UNIT);
		});
	}

	instance = this;
}

uint16_t Controls::getControlFromUSB(uint8_t unit_id, uint8_t control_selector, uint8_t channel, uint8_t bRequest) {
	uint8_t endpoint_index = (unit_id - 1) / AUDIO_UNIT_ID_PER_ENDPOINT;
	uint8_t unit_id_offset = (unit_id - 1) % AUDIO_UNIT_ID_PER_ENDPOINT + 1;
	if(endpoint_index >= controlsMapping.size())
		return 0;

	if(unit_id_offset == AUDIO_UNIT_ID_OFFSET_FEATURE_UNIT) {
		switch(control_selector) {
			case AUDIO_CONTROL_VOLUME:
				switch(bRequest) {
					case AUDIO_REQ_GET_CUR:
						return (uint16_t) (int16_t) (controlsMapping[endpoint_index].volumeControl->getToOsc() * 256);
					case AUDIO_REQ_GET_MIN:
						return (uint16_t) (int16_t) (-127 * 256);
					case AUDIO_REQ_GET_MAX:
						return (uint16_t) (int16_t) (20 * 256);
					case AUDIO_REQ_GET_RES:
						return 256;
				}
				break;

			case AUDIO_CONTROL_MUTE:
				switch(bRequest) {
					case AUDIO_REQ_GET_CUR:
						return controlsMapping[endpoint_index].muteControl->get() ? 1 : 0;
					case AUDIO_REQ_GET_MIN:
						return 0;
					case AUDIO_REQ_GET_MAX:
						return 1;
					case AUDIO_REQ_GET_RES:
						return 1;
				}
				break;
		}
	}
	return 0;
}

void Controls::setControlFromUSB(
    uint8_t unit_id, uint8_t control_selector, uint8_t channel, uint8_t bRequest, uint16_t value) {
	uint8_t endpoint_index = (unit_id - 1) / AUDIO_UNIT_ID_PER_ENDPOINT;
	uint8_t unit_id_offset = (unit_id - 1) % AUDIO_UNIT_ID_PER_ENDPOINT + 1;
	if(endpoint_index >= controlsMapping.size())
		return;

	if(bRequest != AUDIO_REQ_SET_CUR)
		return;

	if(unit_id_offset == AUDIO_UNIT_ID_OFFSET_FEATURE_UNIT) {
		switch(control_selector) {
			case AUDIO_CONTROL_VOLUME:
				controlsMapping[endpoint_index].volumeToSet.value = (int16_t) value / 256.f;
				controlsMapping[endpoint_index].volumeToSet.isChanged = true;
				uv_async_send(&asyncControlChanged);
				break;

			case AUDIO_CONTROL_MUTE:
				controlsMapping[endpoint_index].muteToSet.value = value == 0 ? false : true;
				controlsMapping[endpoint_index].muteToSet.isChanged = true;
				uv_async_send(&asyncControlChanged);
				break;
		}
	}
}
void Controls::onControlChangedStatic(uv_async_t* handle) {
	Controls* thisInstance = (Controls*) handle->data;
	thisInstance->onControlChanged();
}

void Controls::onControlChanged() {
	for(size_t i = 0; i < controlsMapping.size(); i++) {
		if(controlsMapping[i].volumeToSet.isChanged) {
			controlsMapping[i].volumeToSet.isChanged = false;

			// Ensure isChanged is set to false before reading value
			std::atomic_signal_fence(std::memory_order_seq_cst);

			processUsbControlChange = true;
			controlsMapping[i].volumeControl->setFromOsc(controlsMapping[i].volumeToSet.value);
			processUsbControlChange = false;
		}
		if(controlsMapping[i].muteToSet.isChanged) {
			controlsMapping[i].muteToSet.isChanged = false;

			// Ensure isChanged is set to false before reading value
			std::atomic_signal_fence(std::memory_order_seq_cst);

			processUsbControlChange = true;
			controlsMapping[i].muteControl->setFromOsc(controlsMapping[i].muteToSet.value);
			processUsbControlChange = false;
		}
	}
}