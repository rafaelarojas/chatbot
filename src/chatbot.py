import rclpy
from rclpy.node import Node
import re
import unicodedata
from std_msgs.msg import String

# Dicionário de intenções (intenção -> descrição)
intentions = {
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
actions = {
    "secretaria": lambda: "O robô está indo para a secretaria...",
    "laboratorio": lambda: "O robô está indo para o laboratório...",
    "biblioteca": lambda: "O robô está indo para a biblioteca...",
    "atelie": lambda: "O robô está indo para o ateliê...",
    "andre": lambda: "O robô está indo até o André para encher o saco dele...",
    "refeitorio": lambda: "O robô está indo para o refeitório...",
    "recepcao": lambda: "O robô está indo para a recepção...",
    "arquibancada": lambda: "O robô está indo para a arquibancada...",
    "auditorio": lambda: "O robô está indo para o auditório...",
}

# Expressões regulares para compreender variações de comandos
command_patterns = {
    "secretaria": re.compile(r"(va para a secretaria|dirija-se a secretaria|secretaria|me leve para a secretaria)"),
    "laboratorio": re.compile(r"(va para o laboratorio|dirija-se ao laboratorio|laboratorio|me leve para o laboratorio)"),
    "biblioteca": re.compile(r"(va para a biblioteca|dirija-se a biblioteca|biblioteca|me leve para a biblioteca)"),
    "atelie": re.compile(r"(va para o atelie|dirija-se ao atelie|atelie|me leve para o atelie)"),
    "andre": re.compile(r"(va ate o andre encher o saco dele|encontre o andre|va ate o andre)"),
    "refeitorio": re.compile(r"(va para o refeitorio|dirija-se ao refeitorio|refeitorio|me leve para o refeitorio)"),
    "recepcao": re.compile(r"(va para a recepcao|dirija-se a recepcao|recepcao|me leve para a recepcao)"),
    "arquibancada": re.compile(r"(va para a arquibancada|dirija-se a arquibancada|arquibancada|me leve para a arquibancada)"),
    "auditorio": re.compile(r"(va para o auditorio|dirija-se ao auditorio|auditorio|me leve para o auditorio)"),
}

def normalize_text(text):
    """
    Remove acentos e converte o texto para minúsculas.
    """
    return ''.join(
        c for c in unicodedata.normalize('NFKD', text)
        if not unicodedata.combining(c)
    ).lower()

class ChatbotNode(Node):
    def __init__(self):
        super().__init__('chatbot_node')
        self.publisher_ = self.create_publisher(String, 'chatbot_feedback', 10)
        self.get_logger().info("Chatbot do Robô de Serviço iniciado.")
        print("Digite 'sair' para encerrar o programa.")

    def process_command(self, command):
        """
        Processa o comando do usuário e executa a ação correspondente.
        """
        normalized_command = normalize_text(command)
        for intention, pattern in command_patterns.items():
            if pattern.search(normalized_command):
                feedback = actions[intention]()
                self.get_logger().info(f"Intenção: {intentions[intention]}")
                self.get_logger().info(feedback)

                # Publicar feedback no tópico 'chatbot_feedback'
                msg = String()
                msg.data = f"Intenção: {intentions[intention]}\n{feedback}"
                self.publisher_.publish(msg)

                print(f"✔ Comando compreendido: {intentions[intention]}")
                return

        # Caso o comando não seja compreendido
        self.get_logger().info("Desculpe, não entendi o comando.")
        print("✘ Não consegui entender o comando. Tente novamente.")

    def run(self):
        """
        Loop principal do chatbot.
        """
        while rclpy.ok():
            user_input = input("Comando do usuário: ").strip()
            if user_input.lower() == 'sair':
                print("Encerrando o chatbot. Até logo!")
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
