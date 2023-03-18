import pytest
import requests
from unittest import mock
from chatgpt_ros.chatgpt import ChatGPT


@pytest.fixture
def chat_gpt():
    return ChatGPT(api_key="TEST_API_KEY")


@mock.patch.object(requests, "post")
def test_generate_text(mock_post, chat_gpt):
    mock_response_json = {
        "choices": [
            {
                "message": {
                    "content": "This is a test response."
                },
                "index": 0,
                "logprobs": None,
                "finish_reason": "stop",
            }
        ],
        "created": "2022-03-14T01:23:45.678901Z",
        "model": "text-davinci-002",
    }
    mock_post.return_value.json.return_value = mock_response_json
    assert (
        chat_gpt.generate_text("This is a test prompt.")
        == "This is a test response."
    )
    mock_post.assert_called_once_with(
        "https://api.openai.com/v1/chat/completions",
        headers={
            "Content-Type": "application/json",
            "Authorization": "Bearer TEST_API_KEY",
        },
        json={
            "model": "gpt-3.5-turbo",
            "messages": [
                {"role": "user", "content": "This is a test prompt."}
            ],
            "max_tokens": 50,
            "temperature": 0.5,
            "n": 1,
            "stop": ".",
        },
    )


def test_set_assitant_content(chat_gpt):
    chat_gpt.num_hold_pass_res = 2
    chat_gpt.set_assitant_content("This is a test assistant content.")
    assert chat_gpt.assistant_list == [
        {"role": "assistant", "content": "This is a test assistant content."}
    ]
    chat_gpt.set_assitant_content("This is another test assistant content.")
    assert chat_gpt.assistant_list == [
        {"role": "assistant", "content": "This is a test assistant content."},
        {"role": "assistant", "content": "This is another test assistant content."}
    ]
    chat_gpt.set_assitant_content("This is a third test assistant content.")
    assert chat_gpt.assistant_list == [
        {"role": "assistant", "content": "This is another test assistant content."},
        {"role": "assistant", "content": "This is a third test assistant content."}
    ]


def test_set_system_content(chat_gpt):
    chat_gpt.set_system_content("This is a test system message.")
    assert chat_gpt.role_system == {"role": "system", "content": "This is a test system message."}


def test_set_assitant(chat_gpt):
    chat_gpt.set_assitant = True
    assert chat_gpt.set_assitant is True
