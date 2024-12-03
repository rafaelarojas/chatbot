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
    "secretaria": re.compile(r"(vá ?para ?a ?secretaria|dirija-se ?a ?secretaria|secretaria|me ?leve ?para ?a ?secretaria|quero ?ir ?para ?a ?secretaria|secretaria, ?por ?favor|onde ?fica ?a ?secretaria|levar-me ?à ?secretaria|pode ?me ?levar ?à ?secretaria|ir ?até ?a ?secretaria|va ?para ?a ?secretaria)", re.IGNORECASE),
    "laboratorio": re.compile(r"(vá ?para ?o ?laboratório|dirija-se ?ao ?laboratório|laboratório|me ?leve ?para ?o ?laboratório|onde ?fica ?o ?laboratório|laboratório, ?por ?favor|levar-me ?ao ?laboratório|pode ?me ?levar ?ao ?laboratório|ir ?até ?o ?laboratório|quero ?ir ?para ?o ?laboratório|va ?para ?o ?laboratório)", re.IGNORECASE),
    "biblioteca": re.compile(r"(vá ?para ?a ?biblioteca|dirija-se ?a ?biblioteca|biblioteca|me ?leve ?para ?a ?biblioteca|onde ?fica ?a ?biblioteca|quero ?ir ?para ?a ?biblioteca|biblioteca, ?por ?favor|levar-me ?à ?biblioteca|pode ?me ?levar ?à ?biblioteca|ir ?até ?a ?biblioteca|va ?para ?a ?biblioteca)", re.IGNORECASE),
    "atelie": re.compile(r"(vá ?para ?o ?ateliê|dirija-se ?ao ?ateliê|ateliê|me ?leve ?para ?o ?ateliê|onde ?fica ?o ?ateliê|quero ?ir ?para ?o ?ateliê|ateliê, ?por ?favor|levar-me ?ao ?ateliê|pode ?me ?levar ?ao ?ateliê|ir ?até ?o ?ateliê|va ?para ?o ?ateliê)", re.IGNORECASE),
    "andre": re.compile(r"(vá ?até ?o ?andré ?encher ?o ?saco ?dele|encontre ?o ?andré|vá ?até ?o ?andré|ve ?ate ?o ?andre|ir ?ao ?andré|cadê ?o ?andré|me ?leve ?ao ?andré|onde ?está ?o ?andré|ache ?o ?andré|vá ?falar ?com ?o ?andré|falar ?com ?o ?andré|quero ?ver ?o ?andré)", re.IGNORECASE),
    "refeitorio": re.compile(r"(vá ?para ?o ?refeitório|dirija-se ?ao ?refeitório|refeitório|me ?leve ?para ?o ?refeitório|onde ?fica ?o ?refeitório|quero ?ir ?para ?o ?refeitório|refeitório, ?por ?favor|levar-me ?ao ?refeitório|pode ?me ?levar ?ao ?refeitório|ir ?até ?o ?refeitório|va ?para ?o ?refeitório)", re.IGNORECASE),
    "recepcao": re.compile(r"(vá ?para ?a ?recepção|dirija-se ?a ?recepção|recepção|me ?leve ?para ?a ?recepção|onde ?fica ?a ?recepção|quero ?ir ?para ?a ?recepção|recepção, ?por ?favor|levar-me ?à ?recepção|pode ?me ?levar ?à ?recepção|ir ?até ?a ?recepção|va ?para ?a ?recepção)", re.IGNORECASE),
    "arquibancada": re.compile(r"(vá ?para ?a ?arquibancada|dirija-se ?a ?arquibancada|arquibancada|me ?leve ?para ?a ?arquibancada|onde ?fica ?a ?arquibancada|quero ?ir ?para ?a ?arquibancada|arquibancada, ?por ?favor|levar-me ?à ?arquibancada|pode ?me ?levar ?à ?arquibancada|ir ?até ?a ?arquibancada|va ?para ?a ?arquibancada)", re.IGNORECASE),
    "auditorio": re.compile(r"(vá ?para ?o ?auditório|dirija-se ?ao ?auditório|auditório|me ?leve ?para ?o ?auditório|onde ?fica ?o ?auditório|quero ?ir ?para ?o ?auditório|auditório, ?por ?favor|levar-me ?ao ?auditório|pode ?me ?levar ?ao ?auditório|ir ?até ?o ?auditório|va ?para ?o ?auditório)", re.IGNORECASE),
}



def normalize_text(text):
    """
    Remove acentos, converte o texto para minúsculas e normaliza.
    """
    nfkd_form = unicodedata.normalize('NFKD', text)
    no_accents = ''.join([c for c in nfkd_form if not unicodedata.combining(c)])
    
    return no_accents.casefold()

class ChatbotNode(Node):
    def __init__(self):
        super().__init__('chatbot_node')
        self.publisher_ = self.create_publisher(String, 'chatbot_feedback', 10)
        self.get_logger().info("Chatbot do Robô de Serviço iniciado.")
        self.print_welcome_message()

    def print_welcome_message(self):

        print("\033[1;34m" + "=" * 50)
        print("🔹 Bem-vindo ao INCHATBOT CAMPUS🔹")
        print("=" * 50 + "\033[0m")
        print("\033[1;32mDigite 'sair' para encerrar o programa.\033[0m")

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

                print(f"\033[1;32m✔ Comando compreendido: {intentions[intention]}\033[0m")
                print(f"\033[1;34m🔹 {feedback}\033[0m")
                return

        self.get_logger().info("Desculpe, não entendi o comando.")
        print("\033[1;31m✘ Não consegui entender o comando. Tente novamente.\033[0m")

    def run(self):
        """
        Loop principal do chatbot.
        """
        while rclpy.ok():
            user_input = input("\033[1;36mComando do usuário: \033[0m").strip()
            if user_input.lower() == 'sair':
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
