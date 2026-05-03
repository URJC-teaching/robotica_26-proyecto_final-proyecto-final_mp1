# launch/hri_bringup.launch.py
#
# Lanza el stack HRI local:
#   - stt_service_local   : Whisper local    -> /stt_service
#   - tts_service_local   : MMS-TTS local    -> /tts_service
#   - extract_service_hugg: LLM HF cloud     -> /extract_service
#   - yesno_service_local : patrones locales -> /yesno_service
#   - sound_play          : reproduce WAV en el altavoz
#
# Token HuggingFace: se lee de ~/.hf_token (fuera del repo, nunca se sube a git).
# Si no existe ese archivo se intenta la variable de entorno HF_TOKEN.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _read_hf_token() -> str:
    token_file = os.path.expanduser('~/.hf_token')
    if os.path.isfile(token_file):
        return open(token_file).read().strip()
    return os.environ.get('HF_TOKEN', '')


HF_TOKEN = _read_hf_token()


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('HF_TOKEN', HF_TOKEN),
        SetEnvironmentVariable('HUGGINGFACE_HUB_TOKEN', HF_TOKEN),

        DeclareLaunchArgument(
            'tts_lang', default_value='spa',
            description='Código de idioma TTS (spa=español, eng=inglés)'),
        DeclareLaunchArgument(
            'tts_speaks', default_value='true',
            description='True: reproduce audio en altavoz local'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('simple_hri'),
                    'launch',
                    'free_simple_hri.launch.py',
                )
            ),
            launch_arguments={
                'tts_lang':   LaunchConfiguration('tts_lang'),
                'tts_speaks': LaunchConfiguration('tts_speaks'),
            }.items(),
        ),
    ])
