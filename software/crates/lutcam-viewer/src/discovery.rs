//! Find a camera application through the shared read-only Etherbone probe.

use serde::Deserialize;
use std::net::Ipv4Addr;

#[derive(Debug, Deserialize)]
struct Board {
    address: Ipv4Addr,
    image_slot: u32,
    status: u32,
}

fn select_board(output: &[u8]) -> Result<Ipv4Addr, String> {
    let boards: Vec<Board> = serde_json::from_slice(output)
        .map_err(|error| format!("Invalid board discovery response: {error}"))?;
    let mut addresses: Vec<_> = boards
        .into_iter()
        .filter(|board| {
            matches!(board.image_slot, 1 | 2)
                && board.status & 5 == 0
                && !board.address.is_unspecified()
                && !board.address.is_multicast()
                && !board.address.is_broadcast()
        })
        .map(|board| board.address)
        .collect();
    addresses.sort_unstable();
    addresses.dedup();
    match addresses.as_slice() {
        [address] => Ok(*address),
        [] => Err("No LUTCam camera board found. Check power and Ethernet, then launch again; use --board IP for a board outside the local network.".into()),
        _ => Err(format!(
            "Multiple LUTCam boards found ({}). Select one with --board IP.",
            addresses.iter().map(ToString::to_string).collect::<Vec<_>>().join(", ")
        )),
    }
}

pub fn find_board(bind_ip: Ipv4Addr) -> Result<Ipv4Addr, String> {
    let output = crate::helper::command("tools.lutcam.discover")
        .args(["--bind", &bind_ip.to_string(), "--timeout", "0.5"])
        .output()
        .map_err(|error| format!("Could not run board discovery: {error}"))?;
    if !output.status.success() {
        return Err(format!(
            "Board discovery failed: {}",
            String::from_utf8_lossy(&output.stderr).trim()
        ));
    }
    let board = select_board(&output.stdout)?;
    eprintln!("Discovered LUTCam at {board}");
    Ok(board)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn selects_one_application_and_ignores_recovery_and_rebooting_boards() {
        let reply = br#"[
            {"address":"192.0.2.50","image_slot":0,"status":0},
            {"address":"192.0.2.51","image_slot":1,"status":1},
            {"address":"192.0.2.75","image_slot":2,"status":0},
            {"address":"192.0.2.75","image_slot":2,"status":0}
        ]"#;
        assert_eq!(select_board(reply).unwrap(), Ipv4Addr::new(192, 0, 2, 75));
    }

    #[test]
    fn does_not_choose_arbitrarily_between_two_boards() {
        let error = select_board(
            br#"[
            {"address":"192.0.2.76","image_slot":1,"status":0},
            {"address":"192.0.2.75","image_slot":2,"status":0}
        ]"#,
        )
        .unwrap_err();
        assert!(error.contains("192.0.2.75, 192.0.2.76"));
        assert!(error.contains("--board"));
    }

    #[test]
    fn reports_no_board_and_rejects_invalid_results() {
        assert!(select_board(b"[]").unwrap_err().contains("No LUTCam"));
        assert!(select_board(b"not JSON").is_err());
        assert!(
            select_board(br#"[{"address":"255.255.255.255","image_slot":1,"status":0}]"#).is_err()
        );
    }
}
