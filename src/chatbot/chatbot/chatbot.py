import rclpy
from rclpy.node import Node
import re
import unicodedata
from std_msgs.msg import String

# Dicionário de intenções (intenção -> descrição)
INTENTIONS = {
    "secretaria": "Ir para a secretaria",
    "laboratorio": "Ir para o laboratório",
    "biblioteca": "Ir para a biblioteca",
    "atelie": "Ir para o ateliê",
    "andre": "Ir até o André para encher o saco dele",
    "refeitorio": "Ir para o refeitório",
    "recepcao": "Ir para a recepção",
    "arquibancada": "Ir para a arquibancada",
    "auditorio": "Ir para o auditório",
}

# Dicionário de ações (intenção -> função)
ACTIONS = {
    key: lambda desc=desc: f"O robô está indo para {desc.lower()}..." for key, desc in INTENTIONS.items()
}

# Expressões regulares para compreender variações de comandos
COMMAND_PATTERN = re.compile(
    r"(?:va|dirija-se|leve-me|me leve|quero ir|pode me levar|onde fica|ir ate|levar-me)(?:\s+(?:para a|para o))?\s*(\w+)$",
    re.IGNORECASE,
)

def normalize_text(text):
    """
    Remove acentos, converte o texto para minúsculas e normaliza.
    """
    nfkd_form = unicodedata.normalize("NFKD", text)
    no_accents = "".join([c for c in nfkd_form if not unicodedata.combining(c)])
    return no_accents.casefold()

class ChatbotNode(Node):
    def __init__(self):
        super().__init__("chatbot_node")
        self.publisher_ = self.create_publisher(String, "chatbot_feedback", 10)
        self.get_logger().info("Chatbot do Robô de Serviço iniciado.")
        self.print_welcome_message()

    def print_welcome_message(self):
        print("\033[1;34m" + "=" * 50)
        print("🔹 Bem-vindo ao INCHATBOT CAMPUS 🔹")
        print("=" * 50 + "\033[0m")
        print("\033[1;32mDigite 'sair' para encerrar o programa.\033[0m")

    def process_command(self, command):
        """
        Processa o comando do usuário e executa a ação correspondente.
        """
        normalized_command = normalize_text(command)
        match = COMMAND_PATTERN.search(normalized_command)

        if match:
            captured_word = match.group(1).strip()
            if captured_word in INTENTIONS:
                feedback = ACTIONS[captured_word]()
                self.get_logger().info(f"Intenção: {INTENTIONS[captured_word]}")
                self.get_logger().info(feedback)

                # Publicar feedback no tópico 'chatbot_feedback'
                msg = String()
                msg.data = f"Intenção: {INTENTIONS[captured_word]}\n{feedback}"
                self.publisher_.publish(msg)

                print(f"\033[1;32m✔ Comando compreendido: {INTENTIONS[captured_word]}\033[0m")
                print(f"\033[1;34m🔹 {feedback}\033[0m")
                return

        # Caso nenhum comando seja entendido
        self.get_logger().info("Desculpe, não entendi o comando.")
        print("\033[1;31m✘ Não consegui entender o comando. Tente novamente.\033[0m")

    def run(self):
        """
        Loop principal do chatbot.
        """
        while rclpy.ok():
            user_input = input("\033[1;36mComando do usuário: \033[0m").strip()
            if user_input.lower() == "sair":
                print("\033[1;33mEncerrando o chatbot. Até logo! 👋\033[0m")
                self.get_logger().info("Chatbot encerrado pelo usuário.")
                break
            self.process_command(user_input)

def main(args=None):
    rclpy.init(args=args)
    node = ChatbotNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
