use crate::{error::{Error, Result}, types::*};
use reqwest::Client;

#[derive(Debug, Clone)]
pub struct StandardClient {
    api_key: String,
    base_url: String,
    client: Client,
}

impl StandardClient {
    pub fn new(api_key: String) -> Self {
        Self {
            api_key,
            base_url: "https://api.wia.org".to_string(),
            client: Client::new(),
        }
    }
}
