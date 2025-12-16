//! Molecule and atomic structure types

use serde::{Deserialize, Serialize};
use super::Position3D;

/// Chemical element (periodic table)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Element {
    H, He,
    Li, Be, B, C, N, O, F, Ne,
    Na, Mg, Al, Si, P, S, Cl, Ar,
    K, Ca, Sc, Ti, V, Cr, Mn, Fe, Co, Ni, Cu, Zn,
    Ga, Ge, As, Se, Br, Kr,
    Rb, Sr, Y, Zr, Nb, Mo, Tc, Ru, Rh, Pd, Ag, Cd,
    In, Sn, Sb, Te, I, Xe,
    Cs, Ba, La, Ce, Pr, Nd, Pm, Sm, Eu, Gd, Tb, Dy, Ho, Er, Tm, Yb, Lu,
    Hf, Ta, W, Re, Os, Ir, Pt, Au, Hg, Tl, Pb, Bi, Po, At, Rn,
    Fr, Ra, Ac, Th, Pa, U, Np, Pu, Am, Cm, Bk, Cf, Es, Fm, Md, No, Lr,
    Rf, Db, Sg, Bh, Hs, Mt, Ds, Rg, Cn, Nh, Fl, Mc, Lv, Ts, Og,
    #[serde(rename = "custom")]
    Custom(u8),
}

impl Element {
    /// Get atomic number
    pub fn atomic_number(&self) -> u8 {
        match self {
            Element::H => 1, Element::He => 2,
            Element::Li => 3, Element::Be => 4, Element::B => 5,
            Element::C => 6, Element::N => 7, Element::O => 8,
            Element::F => 9, Element::Ne => 10,
            Element::Na => 11, Element::Mg => 12, Element::Al => 13,
            Element::Si => 14, Element::P => 15, Element::S => 16,
            Element::Cl => 17, Element::Ar => 18,
            Element::Fe => 26, Element::Cu => 29, Element::Zn => 30,
            Element::Au => 79, Element::Pt => 78,
            Element::Custom(n) => *n,
            _ => 0, // Simplified for other elements
        }
    }

    /// Get symbol as string
    pub fn symbol(&self) -> &'static str {
        match self {
            Element::H => "H", Element::He => "He",
            Element::C => "C", Element::N => "N", Element::O => "O",
            Element::Fe => "Fe", Element::Au => "Au",
            _ => "X",
        }
    }
}

/// Bond type between atoms
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BondType {
    Single,
    Double,
    Triple,
    Aromatic,
    Hydrogen,
    Ionic,
    VanDerWaals,
    Metallic,
}

/// Atom in a molecule
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Atom {
    pub element: Element,
    pub position: Position3D,
    #[serde(default)]
    pub charge: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub isotope: Option<u16>,
}

impl Atom {
    pub fn new(element: Element, x: f64, y: f64, z: f64) -> Self {
        Self {
            element,
            position: Position3D::new(x, y, z),
            charge: 0.0,
            isotope: None,
        }
    }

    pub fn with_charge(mut self, charge: f64) -> Self {
        self.charge = charge;
        self
    }
}

/// Chemical bond between atoms
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Bond {
    pub atom1_idx: usize,
    pub atom2_idx: usize,
    pub bond_type: BondType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub length_pm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub energy_kj_mol: Option<f64>,
}

impl Bond {
    pub fn new(atom1: usize, atom2: usize, bond_type: BondType) -> Self {
        Self {
            atom1_idx: atom1,
            atom2_idx: atom2,
            bond_type,
            length_pm: None,
            energy_kj_mol: None,
        }
    }

    pub fn single(atom1: usize, atom2: usize) -> Self {
        Self::new(atom1, atom2, BondType::Single)
    }

    pub fn double(atom1: usize, atom2: usize) -> Self {
        Self::new(atom1, atom2, BondType::Double)
    }
}

/// Molecule structure type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StructureType {
    Fullerene,
    Nanotube,
    Protein,
    Dna,
    Rna,
    Polymer,
    Crystal,
    SmallMolecule,
    Custom,
}

/// Complete molecule definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Molecule {
    pub id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    pub formula: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub structure_type: Option<StructureType>,
    pub atoms: Vec<Atom>,
    pub bonds: Vec<Bond>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mass_da: Option<f64>,
    #[serde(default)]
    pub charge: i32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub smiles: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub inchi: Option<String>,
}

impl Molecule {
    pub fn new(id: impl Into<String>, formula: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            name: None,
            formula: formula.into(),
            structure_type: None,
            atoms: Vec::new(),
            bonds: Vec::new(),
            mass_da: None,
            charge: 0,
            smiles: None,
            inchi: None,
        }
    }

    /// Create a C60 Buckminsterfullerene
    pub fn fullerene_c60() -> Self {
        let mut mol = Self::new("C60", "C60");
        mol.name = Some("Buckminsterfullerene".to_string());
        mol.structure_type = Some(StructureType::Fullerene);
        mol.mass_da = Some(720.66);

        // Generate 60 carbon atoms in icosahedral arrangement
        // Simplified - actual positions would be calculated
        for i in 0..60 {
            let theta = (i as f64 / 60.0) * 2.0 * std::f64::consts::PI;
            let phi = (i as f64 / 30.0) * std::f64::consts::PI;
            let r = 3.55; // nm, approximate radius

            mol.atoms.push(Atom::new(
                Element::C,
                r * phi.sin() * theta.cos(),
                r * phi.sin() * theta.sin(),
                r * phi.cos(),
            ));
        }

        mol
    }

    pub fn atom_count(&self) -> usize {
        self.atoms.len()
    }

    pub fn bond_count(&self) -> usize {
        self.bonds.len()
    }

    pub fn center_of_mass(&self) -> Position3D {
        if self.atoms.is_empty() {
            return Position3D::zero();
        }

        let sum = self.atoms.iter().fold(Position3D::zero(), |acc, atom| {
            acc + atom.position
        });

        sum * (1.0 / self.atoms.len() as f64)
    }
}

/// Building block for molecular assembly
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BuildingBlock {
    pub element: String,
    pub count: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub isotope: Option<u16>,
}

impl BuildingBlock {
    pub fn new(element: impl Into<String>, count: u32) -> Self {
        Self {
            element: element.into(),
            count,
            source: None,
            isotope: None,
        }
    }

    pub fn with_source(mut self, source: impl Into<String>) -> Self {
        self.source = Some(source.into());
        self
    }
}
