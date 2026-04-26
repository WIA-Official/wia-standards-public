//! WIA PubScript CLI
//!
//! Command-line tool for parsing, rendering, and converting multi-sensory documents.

use clap::{Parser as ClapParser, Subcommand};
use std::fs;
use std::path::PathBuf;
use wia_pubscript::parser::{markdown::MarkdownParser, Parser};
use wia_pubscript::renderer::{
    braille::BrailleRenderer, html::HtmlRenderer, ssml::SsmlRenderer, Renderer,
};

#[derive(ClapParser)]
#[command(
    name = "wia_pubscript",
    about = "WIA PubScript - Multi-sensory publishing engine",
    version,
    long_about = "Philosophy: ALL FIVE REPRESENTATIONS ARE EQUAL\n\
                  Visual, Auditory, Tactile, Spatial, Gestural\n\
                  NO DEFAULT EXISTS!"
)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Parse a document to IR (Intermediate Representation)
    Parse {
        /// Input file path
        #[arg(value_name = "INPUT")]
        input: PathBuf,

        /// Output file path (JSON)
        #[arg(short, long, value_name = "OUTPUT")]
        output: Option<PathBuf>,

        /// Input format (markdown, html)
        #[arg(short, long, default_value = "markdown")]
        format: String,
    },

    /// Render IR to output format
    Render {
        /// Input IR file (JSON)
        #[arg(value_name = "INPUT")]
        input: PathBuf,

        /// Output format (braille, ssml, html)
        #[arg(short, long, value_name = "FORMAT")]
        format: String,

        /// Output file path
        #[arg(short, long, value_name = "OUTPUT")]
        output: Option<PathBuf>,
    },

    /// Convert directly from source to output format
    Convert {
        /// Input file path
        #[arg(value_name = "INPUT")]
        input: PathBuf,

        /// Output format (braille, ssml, html, all)
        #[arg(short, long, value_name = "FORMAT", default_value = "html")]
        format: String,

        /// Output file path (or directory if --all)
        #[arg(short, long, value_name = "OUTPUT")]
        output: Option<PathBuf>,

        /// Generate all output formats
        #[arg(long)]
        all: bool,
    },
}

fn main() {
    let cli = Cli::parse();

    match cli.command {
        Commands::Parse {
            input,
            output,
            format,
        } => {
            if let Err(e) = handle_parse(&input, output.as_ref(), &format) {
                eprintln!("Error: {}", e);
                std::process::exit(1);
            }
        }
        Commands::Render {
            input,
            format,
            output,
        } => {
            if let Err(e) = handle_render(&input, &format, output.as_ref()) {
                eprintln!("Error: {}", e);
                std::process::exit(1);
            }
        }
        Commands::Convert {
            input,
            format,
            output,
            all,
        } => {
            if let Err(e) = handle_convert(&input, &format, output.as_ref(), all) {
                eprintln!("Error: {}", e);
                std::process::exit(1);
            }
        }
    }
}

fn handle_parse(
    input: &PathBuf,
    output: Option<&PathBuf>,
    format: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("📖 Parsing {} file: {:?}", format, input);

    // Read input
    let content = fs::read_to_string(input)?;

    // Parse to IR
    let doc = match format {
        "markdown" | "md" => {
            let parser = MarkdownParser::new();
            parser.parse(&content)?
        }
        _ => return Err(format!("Unsupported input format: {}", format).into()),
    };

    // Serialize IR to JSON
    let json = serde_json::to_string_pretty(&doc)?;

    // Output
    if let Some(output_path) = output {
        fs::write(output_path, json)?;
        println!("✅ IR written to: {:?}", output_path);
    } else {
        println!("{}", json);
    }

    println!(
        "\n🌟 Parsed {} nodes with FIVE EQUAL representations!",
        doc.content.len()
    );

    Ok(())
}

fn handle_render(
    input: &PathBuf,
    format: &str,
    output: Option<&PathBuf>,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("🎨 Rendering to {} format from: {:?}", format, input);

    // Read IR
    let json = fs::read_to_string(input)?;
    let doc: wia_pubscript::PubScriptDocument = serde_json::from_str(&json)?;

    // Render
    let rendered = match format {
        "braille" | "brf" => {
            let renderer = BrailleRenderer::new();
            renderer.render(&doc)?
        }
        "ssml" => {
            let renderer = SsmlRenderer::new();
            renderer.render(&doc)?
        }
        "html" => {
            let renderer = HtmlRenderer::new();
            renderer.render(&doc)?
        }
        _ => return Err(format!("Unsupported output format: {}", format).into()),
    };

    // Output
    if let Some(output_path) = output {
        fs::write(output_path, &rendered)?;
        println!("✅ Rendered to: {:?}", output_path);
    } else {
        println!("{}", rendered);
    }

    Ok(())
}

fn handle_convert(
    input: &PathBuf,
    format: &str,
    output: Option<&PathBuf>,
    all: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("🔄 Converting: {:?}", input);

    // Read input
    let content = fs::read_to_string(input)?;

    // Parse to IR (assume markdown for now)
    let parser = MarkdownParser::new();
    let doc = parser.parse(&content)?;

    println!("✅ Parsed to IR ({} nodes)", doc.content.len());

    if all {
        // Generate all formats
        let base_name = input.file_stem().unwrap().to_str().unwrap();
        let output_dir = output.map(|p| p.to_path_buf()).unwrap_or_else(|| {
            input
                .parent()
                .unwrap_or_else(|| std::path::Path::new("."))
                .to_path_buf()
        });

        // Braille
        let braille_path = output_dir.join(format!("{}.brf", base_name));
        let braille_renderer = BrailleRenderer::new();
        let braille = braille_renderer.render(&doc)?;
        fs::write(&braille_path, braille)?;
        println!("✅ Braille (점자): {:?}", braille_path);

        // SSML
        let ssml_path = output_dir.join(format!("{}.ssml", base_name));
        let ssml_renderer = SsmlRenderer::new();
        let ssml = ssml_renderer.render(&doc)?;
        fs::write(&ssml_path, ssml)?;
        println!("✅ SSML (TTS):  {:?}", ssml_path);

        // HTML
        let html_path = output_dir.join(format!("{}.html", base_name));
        let html_renderer = HtmlRenderer::new();
        let html = html_renderer.render(&doc)?;
        fs::write(&html_path, html)?;
        println!("✅ HTML:        {:?}", html_path);

        println!("\n🌟 ALL THREE FORMATS ARE EQUAL!");
    } else {
        // Single format
        let rendered = match format {
            "braille" | "brf" => {
                let renderer = BrailleRenderer::new();
                renderer.render(&doc)?
            }
            "ssml" => {
                let renderer = SsmlRenderer::new();
                renderer.render(&doc)?
            }
            "html" => {
                let renderer = HtmlRenderer::new();
                renderer.render(&doc)?
            }
            _ => return Err(format!("Unsupported output format: {}", format).into()),
        };

        if let Some(output_path) = output {
            fs::write(output_path, &rendered)?;
            println!("✅ Converted to: {:?}", output_path);
        } else {
            println!("{}", rendered);
        }
    }

    Ok(())
}
