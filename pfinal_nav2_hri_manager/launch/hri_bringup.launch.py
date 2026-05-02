# launch/hri_bringup.launch.py
#
# Lanza el stack HRI local (sin LLM externo):
#   - stt_service_local  : Whisper local -> /stt_service (SetBool)
#   - tts_service_local  : HuggingFace MMS-TTS local -> /tts_service (Speech)
#   - extract_service_hugg: HF Inference API -> /extract_service (Extract)  [opcional]
#   - yesno_service_local : patrón local -> /yesno_service (YesNo)          [opcional]
#   - sound_play          : reproduce WAV en el altavoz
#
# Los modelos se descargan a ./models/ la primera vez (puede tardar).
# El token de HuggingFace se necesita para extract_service (LLM en la nube).

import os


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from dotenv import load_dotenv


load_dotenv('.')  # Cargar variables de entorno desde el archivo .env

HF_TOKEN = os.getenv('HF_TOKEN', '')  # Obtener el token de HuggingFace (puede ser vacío si no se ha establecido)


def generate_launch_description():
    return LaunchDescription([
        # --- Token HuggingFace para extract_service y descarga de modelos ---
        SetEnvironmentVariable('HF_TOKEN', HF_TOKEN),
        SetEnvironmentVariable('HUGGINGFACE_HUB_TOKEN', HF_TOKEN),

        DeclareLaunchArgument(
            'tts_lang', default_value='spa',
            description='Código de idioma TTS (spa=español, eng=inglés)'),

        DeclareLaunchArgument(
            'tts_speaks', default_value='true',
            description='True: reproduce audio en altavoz local'),

        # --- Lanzar free_simple_hri (STT local + TTS local + extract HF + yesno) ---
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
