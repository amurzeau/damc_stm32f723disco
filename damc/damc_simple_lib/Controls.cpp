#include "Controls.h"
#include "usbd_audio.h"
#include <OscRoot.h>

Controls* Controls::instance;

Controls::Controls(OscRoot* oscRoot) : oscRoot(oscRoot), processUsbControlChange(false) {}

void Controls::init() {
	endpointVolumeControls[0] = (OscReadOnlyVariable<float>*) oscRoot->getNode("strip/0/filterChain/volume");
	endpointVolumeControls[1] = (OscReadOnlyVariable<float>*) oscRoot->getNode("strip/2/filterChain/volume");
	endpointVolumeControls[2] = (OscReadOnlyVariable<float>*) oscRoot->getNode("strip/1/filterChain/volume");
	endpointMuteControls[0] = (OscReadOnlyVariable<bool>*) oscRoot->getNode("strip/0/filterChain/mute");
	endpointMuteControls[1] = (OscReadOnlyVariable<bool>*) oscRoot->getNode("strip/2/filterChain/mute");
	endpointMuteControls[2] = (OscReadOnlyVariable<bool>*) oscRoot->getNode("strip/1/filterChain/mute");

	for(size_t i = 0; i < endpointVolumeControls.size(); i++) {
		endpointVolumeControls[i]->addChangeCallback([this, i](float) {
			if(!processUsbControlChange)
				USBD_AUDIO_NotifyUnitIdChanged(i * AUDIO_UNIT_ID_PER_ENDPOINT + AUDIO_UNIT_ID_OFFSET_FEATURE_UNIT);
		});
	}
	for(size_t i = 0; i < endpointMuteControls.size(); i++) {
		endpointMuteControls[i]->addChangeCallback([this, i](bool) {
			if(!processUsbControlChange)
				USBD_AUDIO_NotifyUnitIdChanged(i * AUDIO_UNIT_ID_PER_ENDPOINT + AUDIO_UNIT_ID_OFFSET_FEATURE_UNIT);
		});
	}

	instance = this;
}

uint16_t Controls::getControlFromUSB(uint8_t unit_id, uint8_t control_selector, uint8_t channel, uint8_t bRequest) {
	uint8_t endpoint_index = (unit_id - 1) / AUDIO_UNIT_ID_PER_ENDPOINT;
	uint8_t unit_id_offset = (unit_id - 1) % AUDIO_UNIT_ID_PER_ENDPOINT + 1;
	if(endpoint_index >= endpointVolumeControls.size())
		return 0;

	if(unit_id_offset == AUDIO_UNIT_ID_OFFSET_FEATURE_UNIT) {
		switch(control_selector) {
			case AUDIO_CONTROL_VOLUME:
				switch(bRequest) {
					case AUDIO_REQ_GET_CUR:
						return (uint16_t) (int16_t) (endpointVolumeControls[endpoint_index]->getToOsc() * 256);
					case AUDIO_REQ_GET_MIN:
						return (uint16_t) (int16_t) (-80 * 256);
					case AUDIO_REQ_GET_MAX:
						return (uint16_t) (int16_t) (20 * 256);
					case AUDIO_REQ_GET_RES:
						return 256;
				}
				break;

			case AUDIO_CONTROL_MUTE:
				switch(bRequest) {
					case AUDIO_REQ_GET_CUR:
						return endpointMuteControls[endpoint_index]->get() ? 1 : 0;
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
	if(endpoint_index >= endpointVolumeControls.size())
		return;

	if(bRequest != AUDIO_REQ_SET_CUR)
		return;

	processUsbControlChange = true;
	if(unit_id_offset == AUDIO_UNIT_ID_OFFSET_FEATURE_UNIT) {
		switch(control_selector) {
			case AUDIO_CONTROL_VOLUME:
				endpointVolumeControls[endpoint_index]->setFromOsc((int16_t) value / 256.f);
				break;

			case AUDIO_CONTROL_MUTE:
				endpointMuteControls[endpoint_index]->setFromOsc(value == 0 ? false : true);
				break;
		}
	}
	processUsbControlChange = false;
}
