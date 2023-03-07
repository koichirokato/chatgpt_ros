import requests

class ChatGPT:
    def __init__(self, api_key):
        self.api_key = api_key
        self.endpoint = "https://api.openai.com/v1/chat/completions"
    
    def generate_text(self, prompt, length=50):
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }
        payload = {
            "model":"gpt-3.5-turbo",
            "messages": [
                    {"role":"user", "content":prompt}
                ],
            "max_tokens": length,
            "temperature": 0.5,
            "n": 1,
            "stop": "."
        }
        response = requests.post(self.endpoint, headers=headers, json=payload)
        response.raise_for_status()
        result = response.json()
        return result["choices"][0]["message"]["content"]
