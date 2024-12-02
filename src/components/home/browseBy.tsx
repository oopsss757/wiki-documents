import React, { useState, useEffect } from 'react';
import css from './form.module.scss'
import clsx from 'clsx'
import 'react-toastify/dist/ReactToastify.css';
import { FaChevronLeft, FaChevronRight } from 'react-icons/fa';
const getIndexImage = (src) => {
  return src && require(`../../../assets/index/browse/${src}`).default;
}

function IndexLatestedViki(props) {
  const [theme, setTheme] = useState(props.theme);
  const wikiList = [
    {
      key: 'edgeAi',
      image: 'edgeAi',
      moreLink:'https://wiki.seeedstudio.com/edge_ai_topic/',
      docList: [
        {
          "path": "/Real%20Time%20Subtitle%20Recoder%20on%20Nvidia%20Jetson",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/speech_subtitle.gif",
          "title": "Speech Subtitle Generation on Nvidia Jetson",
        },
                {
          "path": "/alwaysAI-Jetson-Getting-Started",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/object_detection_alwaysai.gif",
          "title": "Object Detection with AlwaysAI",
        }
        , {
          "path": "/YOLOv8-TRT-Jetson",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/object_detection_yolov8.gif",
          "title": "Deploy YOLOv8 on NVIDIA Jetson using TensorRT",
        },
        {
          "path": "/Lumeo-Jetson-Getting-Started",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/object_detection_lumeo.gif",
          "title": "Object Detection with Lumeo",
        },
        {
          "path": "/a_loam",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/fig0.gif",
          "title": "How to run A-LOAM 3D SLAM on reComputer",
        },
        {
          "path": "/CVEDIA-Jetson-Getting-Started",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/CVEDIA-RT.gif",
          "title": "Getting Started with CVEDIA-RT on NVIDIA® Jetson Devices",
        },
        {
          "path": "/No-code-Edge-AI-Tool",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/no_code_edge_ai_tool.webp",
          "title": "No code Edge AI Tool",
        },
        {
          "path": "https://wiki.seeedstudio.com/How_to_run_local_llm_text_to_image_on_reComputer/",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/local_llm_text_to_image.webp",
          "title": "Run a Local LLM Text-to-Image on reComputer",
        }
      ]
    },
    {
      key: 'arduino',
      image: 'arduino',
      moreLink:'https://wiki.seeedstudio.com/Arduino/',
      docList: [
        {
          "path": "/xiao_esp32s3_getting_started",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/xiaoesp32s3sense.webp",
          "title": "Seeed Studio XIAO ESP32S3 (Sense)",
        },
        {
          "path": "/XIAO_BLE",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/XIAO-nRF52840-Sense-45font.webp",
          "title": "Seeed Studio XIAO nRF52840 (Sense)",
        },
        {
          "path": "/XIAO_ESP32C3_Getting_Started",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/xiao-esp32c3.webp",
          "title": "Seeed Studio XIAO ESP32C3",
        },
        {
          "path": "/xiao_esp32c6_getting_started",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/xiaoc6.webp",
          "title": "Seeed Studio XIAO ESP32C6",
        }, 
        {
          "path": "/Seeeduino-XIAO",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/Seeeduino-XIAO-preview-1.webp",
          "title": "Seeed Studio XIAO SAMD21",
        },
        {
          "path": "/Grove-Beginner-Kit-For-Arduino",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/grove-beginner-kit-for-arduino.webp",
          "title": "Grove Beginner Kit For Arduino",
        },
        {
          "path": "/Wio-Terminal-Getting-Started",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/wio.webp",
          "title": "Wio Terminal",
        }, {
          "path": "/Edgebox-ESP-100-Arduino",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/EdgeBox-ESP-100.webp",
          "title": "EdgeBox-ESP-100",
        }
      ]
    }, {
      key: 'raspberry',
      image: 'raspberry',
      moreLink:'https://wiki.seeedstudio.com/Raspberry_Pi/',
      docList: [
        {
          "path": "/reTerminal",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/reTerminal.webp",
          "title": "Getting Started with reTerminal",
        },
        {
          "path": "/recomputer_r1000_flash_OS",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/reComputer_R1025.webp",
          "title": "reComputer R1000 Raspbian OS to eMMC",
        },
        {
          "path": "/Edge_Box_introduction",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/EdgeBox_RPi_200.webp",
          "title": "Getting Started Edge Box RPI-200",
        }, 
        {
          "path": "/reTerminal-hardware-interfaces-usage",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/HW_overview.webp",
          "title": "Hardware and Interfaces Usage",
        },
        {
          "path": "/pose_based_light_control_with_nodered_and_rpi_with_aikit",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/Node-Red.webp",
          "title": "Pose-Based Light Control with Node-Red and Raspberry Pi with AIkit",
        },
        {
          "path": "/Edgebox-rpi-200-codesys",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/new_project.webp",
          "title": "Codesys for Edge Box RPi 200",
        },
        {
          "path": "/tutorial_of_ai_kit_with_raspberrypi5_about_yolov8n_object_detection",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/yolov8_result.webp",
          "title": "AI Kit with Raspberry Pi 5 about YOLOv8n object detection",
        }, 
        {
          "path": "/reTerminal-DM_intro_FUXA",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/fuxa-editor.webp",
          "title": "Getting Start with FUXA - Web Based SCADA Tool",
        }
      ]
    },
    {
      key: 'nvidia',
      image: 'nvidia',
      moreLink:'https://wiki.seeedstudio.com/NVIDIA_Jetson/',

      docList: [
        {
          "path": "/reServer_Industrial_Getting_Started",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/reserver.webp",
          "title": "Getting Started with reServer Industrial",
        },
        {
          "path": "/reComputer_Industrial_Getting_Started",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/reComputerheadline.webp",
          "title": "Getting Started with reComputer Industrial",
        },
        {
          "path": "/reserver_j501_getting_started",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/reServer_J501.webp",
          "title": "Getting Started with reServer J501",
        },
        {
          "path": "/reComputer_J4012_Flash_Jetpack",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/recomputer-j401.webp",
          "title": "Flash JetPack OS to J401 Carrier Board",
        },
        {
          "path": "/reComputer_Jetson_Series_Initiation",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/rekaiji6.webp",
          "title": "reComputer for Jetson Initiation",
        },
        {
          "path": "/deploy_frigate_on_jetson",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/frigate.webp",
          "title": "Deploy Frigate On NVIDIA Jetson device",
        },
        {
          "path": "/reComputer_Jetson_Memory_Expansion",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/jetson_memory_expansion.webp",
          "title": "reComputer for Jetson Memory Expansion",
        },
        {
          "path": "/YOLOv8-DeepStream-TRT-Jetson",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/yolov8_car.webp",
          "title": "YOLOv8 on Jetson with TensorRT and DeepStream",
        }
      ]
    }, {
      key: 'homeAssistant',
      image: 'ha',
      moreLink:'https://wiki.seeedstudio.com/home_assistant_topic/',
      docList: [
        {
          "path": "/xiao-esp32c3-esphome",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/c3_accesses_ha.webp",
          "title": "XIAO ESP32C3 accesses Home Assistant via ESPHome service",
        },
        {
          "path": "/home_assistant_with_sensecap_lorawan_sensors",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/ha_sensecap.webp",
          "title": "Tango with Home Assistant and SenseCAP Sensors",
        },
        {
          "path": "/SenseCAP_Indicator_Application_Matter",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/indicator-matter-7.webp",
          "title": "SenseCAP Indicator - Matter Application Development",
        },
        {
          "path": "/reterminal-DM-Frigate",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/frigate_with_reterminal.webp",
          "title": "Frigate intergrating with reTerminal DM",
        }, 
                {
          "path": "/Connect-Grove-to-Home-Assistant-ESPHome",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/grove_to_esphome.webp",
          "title": "Connect Grove Modules to Home Assistant using ESPHome",
        },
        {
          "path": "/home_assistant_sensecap",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/config_11.webp",
          "title": "SenseCAP Integration for Home Assistant",
        },
        {
          "path": "/sensecraft_homeassistant_userguide",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/SenseCraft-HomeAssistantimage50.webp",
          "title": "SenseCraft-HomeAssistant User Manual",
        },
        {
          "path": "/ha_xiao_esp32",
          "image": "https://files.seeedstudio.com/wiki/Homepage_browseby/ha-xiao5.webp",
          "title": "XIAO ESP32S3 for Home Assistant",
        }
      ]
    }
  ]
  const [activeIndex, setActiveIndex] = useState(0)
  const [docList, setDocsList] = useState(wikiList[0].docList)
  const [moreLink, setMoreLink] = useState(wikiList[0].moreLink)

  const changeTab = (index) => {
    setActiveIndex(index)
    setDocsList(wikiList[index].docList)
    setMoreLink(wikiList[index].moreLink)
  }
  useEffect(() => {
    const dom = document.querySelector('.navbar__items--right .clean-btn')
    dom && dom.addEventListener('click', () => {
      setTimeout(() => {
        const htmlElement = document.documentElement;
        const dataTheme = htmlElement.getAttribute('data-theme');
        setTheme(dataTheme)
      }, 20);
    });
  }, [props.theme])

  return (
    <div className={clsx(css.browseBy,css[theme], 'com_module')}>
      <div className={css.latested_header}>
        <div className={css.latested_header_left}>
          <div className={css.latested_header_title}>
            Browse by
          </div>
        </div>
      </div>
      <div className={css.browseBy_box}>
        <div className={css.browseBy_header}>
          {wikiList.map((item, index) => {
            const src = getIndexImage(item.image + `${theme === 'light' ? '_light' : ''}.png`)
            return (
              <div className={clsx(css.browseBy_item, index === activeIndex ? css.active : '')} key={item.key} onClick={() => changeTab(index)}>
                <img src={src} alt="" />
              </div>
            )
          })}
        </div>
        <div className={css.browseBy_container}>
          {docList.map((item, index) => {
            return (
              <a className={css.doc_item} href={item.path}>
                <img src={item.image} alt="" />
                <div className={clsx(css.wiki_name,'ellipsis-2')}>{item.title}</div>
              </a>
            )
          })}
        </div>
        <div className={css.browseBy_more}>
          <a href={moreLink}>More  <FaChevronRight size={14}/></a>
        </div>
      </div>
    </div>
  )
}

export default IndexLatestedViki;