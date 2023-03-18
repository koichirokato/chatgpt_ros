import requests


class ChatGPT:
    def __init__(self, api_key):
        '''
        Constructer

        Parameters
        ----------
        api_key : str
            API key for OpenAI
        '''
        self.api_key = api_key
        self.endpoint = 'https://api.openai.com/v1/chat/completions'

        self.role_system = {'role': 'system', 'content': None}
        self.assistant_list = []
        self.num_hold_pass_res = 0
        self.set_assitant = False
        self.set_system = False

    def set_assitant_content(self, content):
        '''
        Set the assistant content of message for the ChatGPT API.

        Parameters
        ----------
        content : str
            The content of the message to set for the assistant.
        '''
        assistant = {'role': 'assistant', 'content': content}
        self.assistant_list.append(assistant)
        diff_num = len(self.assistant_list) - self.num_hold_pass_res
        if diff_num > 0:
            for _ in range(diff_num):
                self.assistant_list.pop(0)
        self.set_assitant = True

    def set_system_content(self, content):
        '''
        Set the system content of message for the ChatGPT API.

        Parameters
        ----------
        content : str
            The content of the message to set for the system.
        '''
        self.role_system['content'] = content
        self.set_system = True

    def generate_text(self, prompt, length=50):
        '''
        Generates text response from ChatGPT API given a prompt.

        Parameters
        ----------
        prompt : str
            The prompt message to initiate the conversation with ChatGPT API.

        length : int, optional
            The maximum number of tokens to generate in the response message.
            Default is 50.

        Returns
        -------
        str
            The generated response text from ChatGPT API.

        Raises
        ------
        requests.exceptions.HTTPError
            If the request to ChatGPT API returns an error status code.
        '''
        messages = [
                    {'role': 'user', 'content': prompt},
                ]
        if self.set_system:
            messages.append(self.role_system)
        if self.set_assitant:
            for assistant in self.assistant_list:
                messages.append(assistant)
        headers = {
            'Content-Type': 'application/json',
            'Authorization': f'Bearer {self.api_key}'
        }
        payload = {
            'model': 'gpt-3.5-turbo',
            'messages': messages,
            'max_tokens': length,
            'temperature': 0.5,
            'n': 1,
            'stop': '.'
        }
        print("message : ", messages)
        response = requests.post(self.endpoint, headers=headers, json=payload)
        response.raise_for_status()
        result = response.json()
        print("result  : ", result)
        result_content = result['choices'][0]['message']['content']
        self.set_assitant_content(result_content)
        return result_content
